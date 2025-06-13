import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import time


class JointTrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('joint_trajectory_action_server')

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'joint_trajectory_position_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        goal = goal_handle.request

        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.joint_names = goal.trajectory.joint_names

        total_points = len(goal.trajectory.points)

        for i, point in enumerate(goal.trajectory.points):
            time.sleep(1)  # Simulate motion duration
            feedback_msg.desired = point
            feedback_msg.actual = point  # Simulate actual = desired
            feedback_msg.error = JointTrajectoryPoint()  # Zero error
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Step {i+1}/{total_points}: Feedback sent.")

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = 0  # SUCCESSFUL
        self.get_logger().info('Goal execution complete.')
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = JointTrajectoryActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
