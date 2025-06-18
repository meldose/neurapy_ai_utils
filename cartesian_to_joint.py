import os # imported os module
import numpy as np # imported numpy module

import rclpy # imported rclpy
from rclpy.node import Node # imported Node
from rclpy.action import ActionClient #imported Actionclient

from control_msgs.action import FollowJointTrajectory # imported FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint # imported JoinTrajectory
from sensor_msgs.msg import JointState #imported Joinstate
from builtin_interfaces.msg import Duration # imported Duration

from ikpy.chain import Chain # imported ik

# created class for URDF handler
class URDFChainHandler:
    def __init__(self, urdf_path: str, base_link: str = "maira7M_root_link"):
        self.urdf_path = urdf_path # defining the urdf path
        self.base_link = base_link # defining the base link
        self.chain = None # defing the chain for ik 

# function for loading chain
    def load_chain(self):
        if not os.path.isfile(self.urdf_path): # checking the urdf path 
            raise FileNotFoundError(f"URDF file not found: {self.urdf_path}") # raise the error 
        self.chain = Chain.from_urdf_file(
            self.urdf_path,
            base_elements=[self.base_link]
        )

        # Hard‐coded printout for indices 2..8 → maira7M_joint1..maira7M_joint7
        print("[URDFChainHandler] Hard coded joint indices:")
        for i in range(2, 9):  # i = 2, 3, ..., 8
            print(f"  {i}: maira7M_joint{i-1}")


    def load_chain(self):

        if not os.path.isfile(self.urdf_path):
            raise FileNotFoundError(f"URDF file not found {self.urdf_path}")
        self.chain=Chain.from_urdf_file(self.urdf_path,base_elements=[self.base_link])


# created function for inverse kinematics

    def inverse_kinematics(self, target_position: np.ndarray,
                           initial_joints: np.ndarray = None) -> np.ndarray:
        if not self.chain:
            raise RuntimeError("Chain is not loaded. Call load_chain() first.")
        if initial_joints is None:
            initial_joints = np.zeros(len(self.chain.links))

        full_solution = self.chain.inverse_kinematics(
            target_position,
            initial_position=initial_joints
        )

        print("[URDFChainHandler] Raw IK solution (one value per link):")
        for i, angle in enumerate(full_solution):
            print(f"  Joint {i} ({self.chain.links[i].name}): {angle:.4f} rad")

        return full_solution

# created class for Cartesian to Joint
class CartesiantoJoint(Node):

# initialise the values
    def __init__(self, joint_names: list[str], goal_angles: np.ndarray, duration: float):
        super().__init__('move_joint_to_joint_client')

# creating and Action client 
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_position_controller/follow_joint_trajectory'
        )
        self._current_joint_state = None
        self._sent_goal = False

        # These are already filtered to only the 7 actuated joints (names & angles).
        self._joint_names = joint_names
        self._goal_angles = goal_angles
        self._duration = duration

        # Subscribe to /joint_states, but we only care about it once (to know current ordering).
        self._js_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

# created function for join state callback
    def _joint_state_callback(self, msg: JointState):
        if self._sent_goal:
            return

        current_names = msg.name # defining the message wiht name 
        current_positions = msg.position # defining the message with position

        # Print “before” vs. “after” for sanity:
        self.get_logger().info("=== BEFORE → CURRENT POSITIONS ===")
        for nm, pos in zip(current_names, current_positions):
            if nm in self._joint_names:
                self.get_logger().info(
                    f"  '{nm}': current = {pos:.4f} rad"
                )

        self.get_logger().info("=== SENDING → GOAL POSITIONS ===") 
        for nm, goal in zip(self._joint_names, self._goal_angles):
            self.get_logger().info(f"  '{nm}': goal = {goal:.4f} rad")

        # Build a JointState just for the 7 joints we want to move:
        js = JointState() # setting the joint states
        js.name = self._joint_names[:]
        js.position = [float(a) for a in self._goal_angles]

        self.send_goal(js, self._duration) # sending the goal to the joinstates
        self._sent_goal = True 
        self.destroy_subscription(self._js_sub)

# function for sending goal ot the robot
    def send_goal(self, goal_joint_state: JointState, duration: float):
        goal_msg = FollowJointTrajectory.Goal() # sending the goal
        goal_msg.trajectory.joint_names = goal_joint_state.name[:]
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()

        point = JointTrajectoryPoint()
        point.positions = list(goal_joint_state.position[:])

        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)

        goal_msg.trajectory.points = [point]

        self.get_logger().info('Waiting for action server...')
        self._client.wait_for_server()

        self.get_logger().info(f'Sending goal (duration={duration:.2f}s)...')
        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)


# setting the function for giving goal response callback
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by the server.')
            return

        self.get_logger().info('Goal accepted → waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

# creating function for feedback callback
    def feedback_callback(self, feedback_msg):
        names = feedback_msg.feedback.joint_names
        desired = feedback_msg.feedback.desired.positions
        actual = feedback_msg.feedback.actual.positions
        for i, nm in enumerate(names):
            self.get_logger().info(
                f"[feedback] '{nm}': desired={desired[i]:.4f}, actual={actual[i]:.4f}"
            )

# creating function for getting result callback
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: error_code = {result.error_code}')
        rclpy.shutdown()

# function to normalize to pi 
def normalize_to_pi(angles: np.ndarray) -> np.ndarray:
    return (angles + np.pi) % (2.0 * np.pi) - np.pi

#  main function :
def main(args=None):
    rclpy.init(args=args)

    urdf_path = "/home/midhun.eldose/neura/sim_ws/src/neura_robot_description/maira7M/urdf/maira7M.urdf"
    handler = URDFChainHandler(urdf_path)
    handler.load_chain()

    target_pos = np.array([-0.5, -0.4, 1.0]) # example to test the  poses[X,Y,Z]
    full_joint_angles = handler.inverse_kinematics(target_pos)
    self.get_logger().info("Succesfully moved to the required position")

    #  Extract only the 7 actuated revolute joints (indices 2..8)
    raw_actuated = full_joint_angles[2 : 2 + 7]

    #  Normalize each to [−π, +π)
    wrapped_actuated = normalize_to_pi(raw_actuated)

    #  Hard‐code the 7 actuated joint names in the exact order:
    actuated_names = [
        "joint1",  # index 2
        "joint2",  # index 3
        "joint3",  # index 4
        "joint4",  # index 5
        "joint5",  # index 6
        "joint6",  # index 7
        "joint7",  # index 8
    ]

    #  Instantiate the MoveJoint client with those 7 hard‐coded names + angles
    duration = 5.0 # setting up the time 
    node = CartesiantoJoint(actuated_names, wrapped_actuated, duration)

    #  Spin until we send the goal:
    rclpy.spin(node)
    rclpy.shutdown()

# callig the main function 
if __name__ == "__main__":
    main()
