import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import AsyncIOExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_commander import MoveGroupCommander
from typing import List, Optional
from tf_transformations import quaternion_from_euler
import time
import asyncio

class MairaKinematics(Node):
    def __init__(self):
        super().__init__('maira_kinematics')
        # Declare a parameter for Cartesian target pose (x, y, z, roll, pitch, yaw)
        self.declare_parameter('cartesian_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Initialize MoveIt! planning group (replace 'arm' with your actual group name)
        self.group = MoveGroupCommander('arm')

        # Set up the FollowJointTrajectory action server
        self._action_server = ActionServer(
            node=self,
            action_type=FollowJointTrajectory,
            action_name='joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request):
        """Accept all incoming trajectory goals."""
        self.get_logger().info('Received new trajectory goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Allow cancellation of the trajectory execution."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    @staticmethod
    def _throw_if_joint_invalid(joint_states: List[float], num_joints: int) -> None:
        """Throw if the joint_states list isn't exactly num_joints long."""
        if isinstance(joint_states, tuple):
            joint_states = list(joint_states)
        if not (isinstance(joint_states, list) and len(joint_states) == num_joints):
            raise TypeError(f"[ERROR] joint_states must be a list of length {num_joints}!")

    @staticmethod
    def _throw_if_pose_invalid(pose: List[float]) -> None:
        """Throw error if given pose is not valid (6-DOF)."""
        if not (isinstance(pose, list) and len(pose) == 6):
            raise TypeError("[ERROR] goal_pose should be a list with length 6!")

    def _get_current_joint_state(self) -> List[float]:
        """Return current joint states.

        Returns
        -------
        List[float]
            Joint states from robot state interface.
        """
        return self._robot_state.getRobotStatus("jointAngles")

    def get_current_joint_state(self) -> List[float]:
        """
        Retrieves the current joint positions of the robot.

        Returns
        -------
        List[float]
            A list of float values representing the current joint angles/positions.
            These are typically expressed in radians or degrees, depending on the robot's configuration.
        """
        # Delegate to internal method for hardware/interface abstraction
        return self._get_current_joint_state()

    def _execute_if_successful(self, id: int) -> bool:
        """Execute trajectory for given id if successful.

        Parameters
        ----------
        id : int
            Id of trajectory to be executed

        Returns
        -------
        bool
            True if execution successful; False otherwise.
        """
        plan_success, execution_feasibility = self._is_id_successful(id)
        self.get_logger().debug(
            f"MairaKinematics::execute_if_successful: plan_success: {plan_success} "
            f"execution_feasibility: {execution_feasibility} id: {id}"
        )
        if plan_success and execution_feasibility:
            self.get_logger().debug("" + "*" * 40)
            self.get_logger().debug(f"execute id {id}")
            self._program.execute([id])
        current = self.get_current_joint_state()
        self.get_logger().info(
            f"After execute id: {id}, joint state is: {current}"
        )
        return plan_success

    async def execute_callback(self, goal_handle):
        """Handle FollowJointTrajectory goals: execute joint trajectory, then a linear Cartesian move."""
        # Log current state
        try:
            current_states = self.get_current_joint_state()
            self.get_logger().info(f"Current joint states: {current_states}")
        except Exception as e:
            self.get_logger().warn(f"Failed to get current joint state: {e}")

        self.get_logger().info('Starting trajectory execution...')
        goal = goal_handle.request
        joint_names = goal.trajectory.joint_names
        num_joints = len(joint_names)

        # Validate all trajectory points
        try:
            for pt in goal.trajectory.points:
                self._throw_if_joint_invalid(pt.positions, num_joints)
        except Exception as e:
            self.get_logger().error(f"Invalid goal: {e}")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return goal_handle.abort(result)

        # Prepare feedback
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names

        prev_time = 0.0
        total = len(goal.trajectory.points)
        for idx, pt in enumerate(goal.trajectory.points):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled by client')
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return goal_handle.cancel(result)

            tfs = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            wait = max(0.0, tfs - prev_time)
            prev_time = tfs
            await asyncio.sleep(wait)

            feedback.desired = pt
            feedback.actual = pt  # simulate perfect tracking
            feedback.error = JointTrajectoryPoint()  # zero error
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Step {idx+1}/{total} feedback published")

        # After trajectory, perform a Cartesian linear move
        cartesian_pose = self.get_parameter('cartesian_pose').get_parameter_value().double_array_value
        self.get_logger().info(f"Executing linear move to pose: {cartesian_pose}")
        try:
            self._throw_if_pose_invalid(cartesian_pose)
            success = self.move_linear(cartesian_pose)
            if not success:
                raise RuntimeError('move_linear returned False')
            self.get_logger().info('Waiting for 1 second after linear move')
            self.wait(1.0)
        except Exception as e:
            self.get_logger().error(f"Error in post-trajectory actions: {e}")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            return goal_handle.abort(result)

        # Succeed the action
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('All actions complete, goal succeeded')
        return goal_handle.succeed(result)

    def move_linear(
        self,
        goal_pose: List[float],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:
        """Execute a Cartesian linear move to goal_pose with optional speed/acceleration."""
        self._throw_if_pose_invalid(goal_pose)
        # Build local kwargs to avoid persistent state
        local_kwargs = {}
        if speed is not None:
            local_kwargs['velocity_scaling_factor'] = speed / 100.0
        if acc is not None:
            local_kwargs['acceleration_scaling_factor'] = acc / 100.0

        # Convert to ROS Pose
        q = quaternion_from_euler(goal_pose[3], goal_pose[4], goal_pose[5])
        ros_pose = Pose()
        ros_pose.position = Point(x=goal_pose[0], y=goal_pose[1], z=goal_pose[2])
        ros_pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        waypoints = [ros_pose]

        try:
            plan, fraction = self.group.compute_cartesian_path(
                waypoints, eef_step=0.01, jump_threshold=0.0, avoid_collisions=True
            )
            self.get_logger().info(f"Move linear fraction: {fraction}")
            if local_kwargs:
                plan = self.group.retime_trajectory(
                    self.group.get_current_state(), plan, **local_kwargs
                )
                self.get_logger().info('Retimed trajectory with scaling factors')
            return bool(self.group.execute(plan, wait=True))
        except Exception as e:
            raise RuntimeError(f"Failed to execute move_linear: {e}")

    @staticmethod
    def wait(time_s: float) -> None:
        """Block for the given time in seconds."""
        time.sleep(time_s)


def main(args=None):
    rclpy.init(args=args)
    node = MairaKinematics()
    executor = AsyncIOExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()