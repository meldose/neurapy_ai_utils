
import rclpy  # imported rclpy module 
from rclpy.node import Node # imported Node module 
from rclpy.action import ActionServer # imported Actionserver module 
from control_msgs.action import FollowJointTrajectory # imported FollowJointTrajecotry module 
from sensor_msgs.msg import JointState # imported Joinstate module 
from trajectory_msgs.msg import JointTrajectoryPoint  # imported JointTrajectoryPoint module 
from std_msgs.msg import Header # imported Header module 
import time # imported time module 
from typing import List, Optional # imported List, Optional ,Tuple module 
import traceback
from geometry_msgs.msg import Pose,PoseStamped

#from neurapy_ai_utils.robot.maira_kinematics import MairaKinematics
from neurapy_ai_utils.functions.utils import init_logger

import sys #imported sys 
sys.path.append("/home/neura/neurapy_alpha/")
import neurapy # imported neurapy 

from neurapy.robot import Robot # importing the neurayp modules

# class for Mairakinematics
class MairaKinematics(Node):

# intialise the values 
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

# defining the function for getting the current joint state
    def _get_current_joint_state(self) -> List[float]:
        return self._robot.get_current_joint_angles()

# defining function to get the current cartesain pose
    def _get_current_cartesian_pose(self) -> List[float]:
        return self._robot.get_current_cartesian_pose()

# function to check if the pose is invalid or not
    def _throw_if_pose_invalid(self, pose: List[float]) -> None:
        if not (isinstance(pose, list) and len(pose) == 6):
            raise TypeError("[ERROR] goal_pose should be a list of 6 floats [x,y,z,rx,ry,rz]")

# function to throw if the list of the poses are invalid or not 
    def _throw_if_list_poses_invalid(self, poses: List[List[float]]) -> None:
        if not (isinstance(poses, list) and all(isinstance(p, list) and len(p) == 6 for p in poses)):
            raise TypeError("[ERROR] goal_poses should be a list of 6-element lists")

# function to wait for motion to finish
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

# function to move linearlly
    def move_linear(
        self,
        goal_pose: List[float],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        rot_speed: Optional[float] = None,
        rot_acc: Optional[float] = None,
    ) -> bool:
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
            self._robot.move_linear(**props, motion_id=motion_id)
            return self.wait_motion_finish(target=goal_pose)
        except Exception:
            self._logger.error(traceback.format_exc())
            return False

# function for move linear through points
    def move_linear_via_points(
        self,
        goal_poses: List[List[float]],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        rot_speed: Optional[float] = None,
        rot_acc: Optional[float] = None,
        blending_radius: Optional[float] = None,
    ) -> bool:
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

# class TestjointTrajectoryServer

class TestJointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('test_joint_trajectory_server')
        self._logger = self.get_logger()

        # Load parameters
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

        # Publisher for current Cartesian TCP pose
        self._pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)

        # Subscriber to joint states, to compute & publish Cartesian
        self._waypoints: List[List[float]] = []

# subscriber and Publisher

        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state, 10)

        self._latest_state: Optional[JointState] = None
        self._logger.info("TestJointTrajectoryServer initialized.")

    def joint_state(self, msg: JointState):
        """Convert each JointState → Cartesian TCP pose; publish & record it."""
        self._latest_state = msg

        try:
            cart = self._maira_kin._get_current_cartesian_pose()
        except Exception as e:
            self._logger.error(f"Failed to get Cartesian pose: {e}")
            return

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'base_link'
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = cart[0], cart[1], cart[2]
        qx, qy, qz, qw = quaternion_from_euler(cart[3], cart[4], cart[5])
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        # publish current Cartesian pose
        self._pose_pub.publish(ps)
        # record for later replay
        self._waypoints.append(cart)

# callback function 
    def _execute_callback(self, goal_handle):
        self._logger.info('Replaying recorded Cartesian path...')
        success = False

        try:
            # if only one pose was recorded, do a straight‐linear move
            if len(self._waypoints) == 1:
                target = self._waypoints[0]
                self._logger.debug(f' → Single‐pose move to {target}')
                success = self._maira_kin.move_linear(target)

            # if multiple were recorded, do a via‐points move
            elif len(self._waypoints) > 1:
                self._logger.debug(f' → Multi‐pose move via {len(self._waypoints)} points')
                success = self._maira_kin.move_linear_via_points(self._waypoints)

            else:
                # no waypoints? that’s an error
                self._logger.error('No waypoints recorded!')
                success = False

        except Exception:
            self._logger.error('Exception during Cartesian replay:')
            self._logger.error(traceback.format_exc())
            success = False

        result = FollowJointTrajectory.Result()
        if success:
            self._logger.info('Motion replay succeeded')
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
        else:
            self._logger.error('Motion replay failed')
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()

        # clear for next run
        self._waypoints = []
        return result
# function to destroy the node 

    def destroy_node(self):
        self._logger.info('Shutting down server')
        if hasattr(self._maira_kin, 'finish'):
            self._maira_kin.finish()
        super().destroy_node()

# main function 
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

# calling the main function 
if __name__ == '__main__':
    main()
