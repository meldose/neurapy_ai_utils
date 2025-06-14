import rclpy  # imported rclpy module 
from rclpy.node import Node # imported Node module 
from geometry_msgs.msg import Pose # imported Pose module 
from sensor_msgs.msg import JointState # imported Jointstate module 
from control_msgs.action import FollowJointTrajectory # imported FollowJoitnTrajecotry module 
from rclpy.action import ActionServer # imported Actionserver module 

# class cartesianto Joint Action Server 
class CartesianToJointActionServer(Node):

    def __init__(self):
        super().__init__('cartesian_to_joint_action_server')

        # Subscriber to Cartesian pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/cartesian_pose',
            self.cartesian_pose_callback,
            10
        )

        # Publisher for joint positions
        self.joint_publisher = self.create_publisher(JointState, '/joint_positions', 10)

        # Action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "joint_trajectory_position_controller/follow_joint_trajectory",
            self.execute_callback
        )

# function for cartesian pose callback 
    def cartesian_pose_callback(self, pose_msg:Pose):
        self.get_logger().info(f'Received Cartesian Pose: {pose_msg}')

        # Convert Cartesian pose to joint positions
        joint_positions = self.kinematics.cartesian_to_joint(pose_msg)

        if joint_positions:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.position = joint_positions

            self.joint_publisher.publish(joint_state_msg)
            self.get_logger().info(f"Published joint positions are: {joint_positions}")
        else:
            self.get_logger().error('Cartesian to Joint conversion failed')

    # function for executing callback 
    def execute_callback(self, goal_handle):
            """Execute the trajectory following action.
            
            Parameters
            ----------
            goal_handle : ServerGoalHandle
                The goal handle containing the trajectory to execute
            """
            self._logger.info("Executing trajectory")
            
            trajectory = goal_handle.request.trajectory
            
            # Basic validation
            if not trajectory.points:
                self._logger.error("Empty trajectory received")
                goal_handle.abort()
                return FollowJointTrajectory.Result()
                
            if len(trajectory.joint_names) != self._maira_kinematics.num_joints:
                self._logger.error(f"Joint names mismatch. Expected {self._maira_kinematics.num_joints}, got {len(trajectory.joint_names)}")
                goal_handle.abort()
                return FollowJointTrajectory.Result()
            

            # Convert trajectory to joint positions
            joint_positions = []
            for point in trajectory.points:
                if len(point.positions) != self._maira_kinematics.num_joints:
                    self._logger.error(f"Point has {len(point.positions)} positions, expected {self._maira_kinematics.num_joints}")
                    goal_handle.abort()
                    return FollowJointTrajectory.Result()
                joint_positions.append(list(point.positions))
            
            self._logger.info(f"Executing trajectory with {len(joint_positions)} points")
            
            try:
                # Execute trajectory
                if len(joint_positions) == 1:
                    # Single point trajectory
                    success = self._maira_kinematics.cartesian_to_joint (joint_positions[0]) 
                
                result = FollowJointTrajectory.Result()
                    
                
                if success:
                    self._logger.info("Trajectory execution completed successfully")
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    goal_handle.succeed()
                else:
                    self._logger.error("Trajectory execution failed")
                    result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                    goal_handle.abort()
                    
                return result
                
            except Exception as e:
                self._logger.error(f"Exception during trajectory execution: {e}")
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                goal_handle.abort()
                return result

# main function 
def main(args=None):
    rclpy.init(args=args)

    node = CartesianToJointActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

# callig main function
if __name__ == '__main__':
    main()
