import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import traceback
from typing import List, Optional

from neurapy.robot import Robot


class MairaKinematics(Node):
    def __init__(
        self,
        joint_names_param: List[str],
        id_manager: int = 0,
        speed_move_joint: int = 20,
        speed_move_linear: float = 0.1,
        rot_speed_move_linear: float = 0.87266463,
        acc_move_joint: int = 20,
        acc_move_linear: float = 0.1,
        rot_acc_move_linear: float = 1.74532925,
        blending_radius: float = 0.005,
        require_elbow_up: bool = True,
    ):
        super().__init__('maira_kinematics')
        self._logger = self.get_logger()

        # Joint names
        if not joint_names_param:
            self._joint_names = [f'joint{i+1}' for i in range(7)]
            self._logger.info("Using default joint names")
        else:
            self._joint_names = joint_names_param
        self.num_joints = len(self._joint_names)

        # Motion parameters
        self.id_manager = id_manager
        self.speed_move_joint = speed_move_joint
        self.speed_move_linear = speed_move_linear
        self.rot_speed_move_linear = rot_speed_move_linear
        self.acc_move_joint = acc_move_joint
        self.acc_move_linear = acc_move_linear
        self.rot_acc_move_linear = rot_acc_move_linear
        self.blending_radius = blending_radius
        self.require_elbow_up = require_elbow_up

        # Robot interface
        self._robot = Robot()
        self._logger.info("[MairaKinematics]: initialization complete.")

    def _get_current_joint_state(self) -> List[float]:
        return self._robot.get_current_joint_angles()

    def _get_current_cartesian_pose(self) -> List[float]:
        return self._robot.get_current_cartesian_pose()

    def _throw_if_pose_invalid(self, pose: List[float]) -> None:
        if not (isinstance(pose, list) and len(pose) == 6):
            raise TypeError("[ERROR] goal_pose should be a list of 6 floats [x,y,z,rx,ry,rz]")

    def _throw_if_list_poses_invalid(self, poses: List[List[float]]) -> None:
        if not (isinstance(poses, list) and all(isinstance(p, list) and len(p) == 6 for p in poses)):
            raise TypeError("[ERROR] goal_poses should be a list of 6-element lists")

    def wait_motion_finish(
        self,
        target: List[float],
        threshold: float = 0.01,
        timeout: float = 50.0,
        rate: float = 10.0
    ) -> bool:
        start_time = time.time()
        period = 1.0 / rate

        while time.time() - start_time < timeout:
            current = self._get_current_cartesian_pose()
            if all(abs(c - t) <= threshold for c, t in zip(current, target)):
                return True
            time.sleep(period)
        return False

    def move_linear(
        self,
        goal_pose: List[float],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        rot_speed: Optional[float] = None,
        rot_acc: Optional[float] = None,
    ) -> bool:
        """Execute a Cartesian linear move to the specified pose."""
        self._throw_if_pose_invalid(goal_pose)

        self.id_manager += 1
        motion_id = self.id_manager

        props = {
            "target_pose": goal_pose,
            "speed": self.speed_move_linear if speed is None else speed,
            "acceleration": self.acc_move_linear if acc is None else acc,
            "rotational_speed": self.rot_speed_move_linear if rot_speed is None else rot_speed,
            "rotational_acceleration": self.rot_acc_move_linear if rot_acc is None else rot_acc,
            "blend_radius": 0.0,
        }

        try:
            # Use the robot’s Cartesian move API
            self._robot.move_linear(**props, motion_id=motion_id)
            return self.wait_motion_finish(target=goal_pose)
        except Exception:
            self._logger.error(traceback.format_exc())
            return False

    def move_linear_via_points(
        self,
        goal_poses: List[List[float]],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        rot_speed: Optional[float] = None,
        rot_acc: Optional[float] = None,
        blending_radius: Optional[float] = None,
    ) -> bool:
        """Execute a Cartesian linear move through multiple poses."""
        self._throw_if_list_poses_invalid(goal_poses)
        self.id_manager += 1
        motion_id = self.id_manager

        waypoints = [self._get_current_cartesian_pose()] + goal_poses
        props = {
            "target_pose": waypoints,
            "speed": self.speed_move_linear if speed is None else speed,
            "acceleration": self.acc_move_linear if acc is None else acc,
            "rotational_speed": self.rot_speed_move_linear if rot_speed is None else rot_speed,
            "rotational_acceleration": self.rot_acc_move_linear if rot_acc is None else rot_acc,
            "blend_radius": self.blending_radius if blending_radius is None else blending_radius,
        }

        try:
            self._robot.move_linear(**props, motion_id=motion_id)
            return self.wait_motion_finish(target=goal_poses[-1])
        except Exception:
            self._logger.error(traceback.format_exc())
            return False


class TestJointTrajectoryServer(Node):
    """ROS2 Action Server that dispatches Cartesian linear moves based on FollowJointTrajectory requests."""
    def __init__(self):
        super().__init__('test_joint_trajectory_server')
        self._logger = self.get_logger()

        # Parameters
        self.declare_parameter('joint_names',
                               ['joint1','joint2','joint3','joint4','joint5','joint6','joint7'])
        self.declare_parameter('action_name', 'follow_joint_trajectory')
        joint_names = self.get_parameter('joint_names')\
                          .get_parameter_value().string_array_value
        action_name = self.get_parameter('action_name')\
                         .get_parameter_value().string_value

        # Kinematics & action server
        self._maira_kin = MairaKinematics(joint_names_param=joint_names)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            self._execute_callback
        )

        # Joint state publisher at 50Hz
        self._state_pub = self.create_publisher(JointState, '/internal_kin_joint_states', 10)
        self.create_timer(0.02, self._publish_joint_states)

        self._logger.info("Simple Joint Trajectory Action Server initialized.")

    def _execute_callback(self, goal_handle):
        self._logger.info('Received trajectory goal.')
        traj = goal_handle.request.trajectory

        # Validate
        if not traj.points:
            self._logger.error('Empty trajectory')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        if len(traj.joint_names) != self._maira_kin.num_joints:
            self._logger.error('Joint count mismatch')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Convert to list-of-lists
        poses = [list(p.positions) for p in traj.points]

        # Dispatch to linear motion
        try:
            if len(poses) == 1:
                success = self._maira_kin.move_linear(poses[0])
            else:
                success = self._maira_kin.move_linear_via_points(poses)

            result = FollowJointTrajectory.Result()
            if success:
                self._logger.info('Motion succeeded')
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
            else:
                self._logger.error('Motion failed')
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                goal_handle.abort()
            return result

        except Exception:
            self._logger.error(traceback.format_exc())
            goal_handle.abort()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return res

    def _publish_joint_states(self):
        try:
            positions = self._maira_kin._get_current_joint_state()
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self._maira_kin._joint_names
            msg.position = positions
            msg.velocity = [0.0] * len(positions)
            msg.effort = [0.0] * len(positions)
            self._state_pub.publish(msg)
        except Exception:
            self._logger.error(traceback.format_exc())

    def destroy_node(self):
        self._logger.info('Shutting down server')
        if hasattr(self._maira_kin, 'finish'):
            self._maira_kin.finish()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    server = TestJointTrajectoryServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
