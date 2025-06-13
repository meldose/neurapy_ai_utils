import rclpy #imported rclpy module
from rclpy.node import Node # imported Node module 

from example_interfaces.action import Fibonacci  # Use your custom action if needed
from rclpy.action import ActionServer # imported Action server 
from rclpy.action import CancelResponse, GoalResponse # imported Cancel Reponse and Goal response 

import time #imported time module 


# class FibonaaciAction Server 
class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

# function for goal callback 
    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

# function for cancel callback 
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        result = []
        a, b = 0, 1

        feedback_msg = Fibonacci.Feedback()
        for i in range(goal_handle.request.order):
            result.append(a)
            a, b = b, a + b

            feedback_msg.partial_sequence = result
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Published feedback: {result}')
            time.sleep(1.0)

        goal_handle.succeed()

        result_msg = Fibonacci.Result()
        result_msg.sequence = result
        return result_msg

# main function
def main(args=None):
    rclpy.init(args=args)
    action_server = FibonacciActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()
