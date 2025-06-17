
# import rclpy  # imported rclpy module 
# from rclpy.node import Node # imported Node module 
# from rclpy.action import ActionServer, CancelResponse, GoalResponse # imported CancelResponse and GoalResponse module 
# from geometry_msgs.msg import PoseStamped # imported Posestamped module 
# from sensor_msgs.msg import JointState # imported Joinstate module 
# from std_msgs.msg import Bool 
# from control_msgs.action import FollowJointTrajectory  
# from trajectory_msgs.msg import JointTrajectoryPoint 
# from typing import List, Optional 
# import time # imported time module

# # class Mairakinematic
# class MairaKinematics:

#     def __init__(self):
#         self.num_joints = 7
#         self.joint_names = [f'joint{i+1}' for i in range(self.num_joints)]
#         self._current_joint_state: Optional[List[float]] = None

#     def cartesian_to_joint(self, pose_msg: PoseStamped) -> Optional[List[float]]:
                
#         """Get the inverse kinematics for a given pose and current joint state

#         Parameters
#         ----------
#         goal_pose_cartesian : List[float]
#             Cartesian pose of tcp frame
#         reference_joint_states : Optional[List[float]], optional
#             Joint positions to seed IK solver. Default is None, to use current
#             joint positions.

#         Returns
#         -------
#         List[float]
#             Joint positions from IK solution.

#         Raises
#         ------
#         TypeError
#             Wrong input type

#         ValueError
#             IK solver failed

#         """

#         pos = pose_msg.pose.position
#         ori = pose_msg.pose.orientation
#         goal = [pos.x, pos.y, pos.z,
#                 ori.x, ori.y, ori.z, ori.w]

# class CartesianToJointActionServer(Node):

#     def __init__(self):
#         super().__init__('cartesian_to_joint_action_server')
#         self.get_logger().info('Initializing Cartesian→Joint action server')

#         # Kinematics solver
#         self._kin = MairaKinematics()

#         # Publisher for IK-generated joint states
#         self._joint_pub = self.create_publisher(JointState, '/ik_joint_states', 10)

#         # Publisher for joint-state-received flag
#         self._flag_pub = self.create_publisher(Bool, '/joint_state_received_flag', 10)

#         # Subscriber for Cartesian pose commands
#         self._pose_sub = self.create_subscription(
#             PoseStamped,
#             '/cmd_pose',
#             self.on_pose_msg,
#             10,
#         )

#         # Subscriber for robot’s actual joint states with sensor QoS
#         self._joint_state_sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state,
#             10,
#         )
#         self._latest_state: Optional[JointState] = None

#         # FollowJointTrajectory action server
#         self._action_server = ActionServer(
#             node=self,
#             action_type=FollowJointTrajectory,
#             action_name='joint_trajectory_position_controller/follow_joint_trajectory',
#             execute_callback=self.execute_callback,
#             goal_callback=self.goal_callback,
#             cancel_callback=self.cancel_callback,
#         )

#     def on_pose_msg(self, msg: PoseStamped) -> None:
#         self.get_logger().info('Received Cartesian pose')
#         joint_positions = self._kin.cartesian_to_joint(msg)
#         if joint_positions is None:
#             self.get_logger().error('IK failed: could not compute joint positions')
#             return

#         js = JointState()
#         js.header.stamp = self.get_clock().now().to_msg()
#         js.header.frame_id = 'base_link'
#         js.name = self._kin.joint_names
#         js.position = joint_positions
#         self._joint_pub.publish(js)
#         self.get_logger().info(f'Published IK joint positions: {joint_positions}')

#     def joint_state(self, msg: JointState) -> None:
#         self._latest_state = msg
#         self._kin._current_joint_state = list(msg.position)
#         pos_str = ', '.join(f'{n}={p:.3f}' for n, p in zip(msg.name, msg.position))
#         self.get_logger().debug(f'Received joint states → {pos_str}')

#         flag = Bool()
#         flag.data = True
#         self._flag_pub.publish(flag)

#     def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
#         self.get_logger().info('Received FollowJointTrajectory goal request')
#         return GoalResponse.ACCEPT

#     def cancel_callback(self, goal_handle) -> CancelResponse:
#         self.get_logger().info('Cancel request received')
#         return CancelResponse.ACCEPT

#     def execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
#         self.get_logger().info('Executing trajectory')
#         traj = goal_handle.request.trajectory
#         n = self._kin.num_joints

#         # Validate joint names and points
#         if len(traj.joint_names) != n:
#             self.get_logger().error(f'Expected {n} joints, got {len(traj.joint_names)}')
#             goal_handle.abort()
#             result = FollowJointTrajectory.Result()
#             result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
#             return result

#         for idx, pt in enumerate(traj.points):
#             if len(pt.positions) != n:
#                 self.get_logger().error(f'Point #{idx} wrong size: {len(pt.positions)}')
#                 goal_handle.abort()
#                 res = FollowJointTrajectory.Result()
#                 res.error_code = FollowJointTrajectory.Result.INVALID_GOAL
#                 return res

#         # Simulate execution with feedback
#         prev_t = 0.0
#         for idx, pt in enumerate(traj.points):
#             t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
#             dt = t - prev_t
#             if dt > 0.0:
#                 time.sleep(dt)
#             feedback = FollowJointTrajectory.Feedback()
#             feedback.joint_names = traj.joint_names
#             feedback.desired = pt
#             feedback.actual = pt
#             feedback.error = JointTrajectoryPoint()
#             goal_handle.publish_feedback(feedback)
#             self.get_logger().debug(f'Sent feedback for point {idx}')
#             prev_t = t

#         # Finished
#         result = FollowJointTrajectory.Result()
#         result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
#         goal_handle.succeed()
#         self.get_logger().info('Trajectory execution completed successfully')
#         return result

# # calling the main function 
# def main(args=None):
#     rclpy.init(args=args)
#     server = CartesianToJointActionServer()
#     try:
#         rclpy.spin(server)
#     except KeyboardInterrupt:
#         server.get_logger().info('Shutting down...')
#     finally:
#         server.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy  # ROS2 Python client library
from rclpy.node import Node # imported Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse # imported Actionserver
from sensor_msgs.msg import JointState # imported Jointstate
from std_msgs.msg import Bool # imported Bool
from control_msgs.action import FollowJointTrajectory # imported Joint Trajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from typing import List, Optional
import cmd_interface as cmd  # hypothetical command interface module

# created class for Mairakinematics

class MairaKinematics:

    def __init__(self,speed_move_joint=1.0, acc_move_joint=1.0):
        self.num_joints = 7
        self.joint_names = [f'joint{i+1}' for i in range(self.num_joints)]
        self._current_joint_state: Optional[List[float]] = None
        self.speed_move_joint = speed_move_joint
        self.acc_move_joint = acc_move_joint

    def move_joint_to_joint(
        self,
        goal_pose: List[float],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:
        self.throw_if_joint_invalid(goal_pose)

        props = {
            'target_joint': [goal_pose],
            'speed': self.speed_move_joint if speed is None else speed,
            'acceleration': self.acc_move_joint if acc is None else acc,
            'interpolator': 1,
            'enable_blending': True,
        }

        plan_id = self._id_manager.update_id()
        self._program.set_command(
            cmd.Joint,
            **props,
            cmd_id=plan_id,
            current_joint_angles=self.get_current_joint_state(),
            reusable_id=0,
        )

        return self.execute_if_successful(plan_id)

    def throw_if_joint_invalid(self, pose: List[float]):
        expected = self.num_joints
        if not isinstance(pose, list) or len(pose) != expected:
            raise TypeError(f"Expected {expected} joint values, got {pose}")

    def get_current_joint_state(self) -> List[float]:
        if self._current_joint_state is None:
            raise RuntimeError("Current joint state unknown")
        return self._current_joint_state

    def execute_if_successful(self, id: int) -> bool:
        success = self._program.execute_plan(id)
        if not success:
            raise RuntimeError(f"Execution of plan {id} failed")
        return True

# crearted class for Action server 
class CartesianToJointActionServer(Node):

    def __init__(self):
        super().__init__('cartesian_to_joint_action_server')
        self.get_logger().info('Initializing action server')

        # Kinematics + motion interface
        self._kin = MairaKinematics()

        # Subscriber for joint command requests
        self._joint_sub = self.create_subscription(
            JointState,
            '/cmd_joint',
            self.on_joint_cmd,
            10,
        )

        # FollowJointTrajectory action server (optional)
        self._action_server = ActionServer(
            node=self,
            action_type=FollowJointTrajectory,
            action_name='joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Track latest state from sensors
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10,
        )

    def on_joint_cmd(self, msg: JointState) -> None:
        self.get_logger().info('Received joint command request')
        try:
            success = self._kin.move_joint_to_joint(list(msg.position))
            print(success)
        except Exception as e:
            self.get_logger().error(f'Motion failed: {e}')
            return
        self.get_logger().info('Motion command completed successfully')


    def joint_state_cb(self, msg: JointState) -> None:
        self._kin._current_joint_state = list(msg.position)

    def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        self.get_logger().info('FollowJointTrajectory goal received')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        self.get_logger().info('Executing trajectory…')
        traj = goal_handle.request.trajectory
        n = self._kin.num_joints

        # Basic validation
        if len(traj.joint_names) != n:
            self.get_logger().error('Invalid # of joints')
            goal_handle.abort()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return res

        for idx, pt in enumerate(traj.points):
            if len(pt.positions) != n:
                self.get_logger().error('Point length mismatch')
                goal_handle.abort()
                r = FollowJointTrajectory.Result()
                r.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return r

        # Sequentially execute each waypoint
        prev_t = 0.0
        for idx, pt in enumerate(traj.points):
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dt = t - prev_t
            if dt > 0:
                self.move_delay(dt)
            feedback = FollowJointTrajectory.Feedback(
                joint_names=traj.joint_names,
                desired=pt,
                actual=pt,
                error=JointTrajectoryPoint(),
            )
            goal_handle.publish_feedback(feedback)
            prev_t = t

        # Success
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def move_delay(self, dt: float):
        # simple blocking delay; consider a non-blocking timer for production
        import time
        time.sleep(dt)


def main(args=None):
    rclpy.init(args=args)
    server = CartesianToJointActionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down')
    finally:
        server.destroy_node()
        rclpy.shutdown()
