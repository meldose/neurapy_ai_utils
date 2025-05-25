import neurapy_ai.utils.ros_conversions as rc # imported the ros conversions as rc 
import numpy as np # imported numpy module
import rospy # imported rospy module
from neurapy_ai.clients.autonomous_pick_client import (
    AutonomousPickClient,
)
from neurapy_ai.clients.data_based_pick_client import (
    DataBasedPickClient,
)
from neurapy_ai.clients.pointing_pick_client import PointingPickClient
from neurapy_ai.experimental.clients.object_manipulation_client import (
    ObjectManipulationClient,
)
from neurapy_ai.experimental.clients.stable_poses_generator_client import (
    StablePoseGeneratorClient,
)
from neurapy_ai.utils.types import ObjectWithPose, Pose
from visualization_msgs.msg import Marker, MarkerArray

from neurapy_ai_utils.robot.maira_kinematics import MairaKinematics
from neurapy_ai_utils.robot.robot import Robot

robot = Robot(MairaKinematics()) # setting up the Robot mairakinematics 

# creating function for move joint
def mj(js):
    robot.move_joint_to_joint(js, speed=50, acc=20)

# created function for move linear
def ml(pt):
    robot.move_linear(pt, 0.25, 0.1)

# created function for publishing the pose 
def publish_pose(pose, title):
    print(title + "!!!!!!!!!")
    pose_msg = rc.pose_2_geometry_msg_pose(pose)
    debug_marker = Marker()
    debug_marker.header.frame_id = "root_link"
    debug_marker.action = debug_marker.ADD
    debug_marker.type = debug_marker.MESH_RESOURCE
    debug_marker.mesh_resource = mesh_path
    debug_marker.ns = "object_pose"
    debug_marker.id = 1
    debug_marker.scale.x = 0.001
    debug_marker.scale.y = 0.001
    debug_marker.scale.z = 0.001
    debug_marker.color.r = 0.6
    debug_marker.color.g = 0.6
    debug_marker.color.b = 0.6
    debug_marker.color.a = 0.6
    debug_marker.pose = pose_msg
    marker_array = MarkerArray()
    marker_array.markers.append(debug_marker)
    debug_pub.publish(marker_array)
    rospy.sleep(2)
    return


rospy.init_node("test_node") # intialise the node 
debug_pub = rospy.Publisher(
    "obj_manipulation_debug", MarkerArray, latch=True, queue_size=3
) # creating an publisher
mesh_path = (
    "file:///home/neura_ai/data/objects/puzzle_trapezoid/puzzle_trapezoid.stl"
)


workspace_name = "test_table"
bin_name = ""
gripper_name = "RobotiQ"
# object_names = ["neura_rectangle", ]
object_names = [
    "puzzle_trapezoid",
]
# object_names = [ ]

# setting up the class for AP;DP;PP;SPG

AP = AutonomousPickClient()
print("###############")
DP = DataBasedPickClient()
print("###############")
PP = PointingPickClient()
print("###############")
SPG = StablePoseGeneratorClient()
print("###############")

return_code = DP.start_detection(
    object_names, workspace_name, bin_name, gripper_name
) # return the code having the detection 
return_code, start_picks = DP.get_picks()


pick = start_picks[0]
print("pick:!!!!!!!!!")
print(pick.object_with_pose.pose.orientation)
print(pick.object_with_pose.pose.translation)
# print("1",pick.object.pose.orientation, pick.object.pose.translation)
# ObjectWithPose("puzzle_trapezoid", Pose([0.8619932598242034, -0.37368587671781706, 0.40071055084661766], [0.6460406389209613, -0.10671023975685137, 0.2159253522158627, 0.7243069551196386]))
start_pose = pick.object 

# creating the end pose with the object trapezoid 
end_pose = ObjectWithPose(
    "puzzle_trapezoid",
    Pose(
        [0.9253371149304181, 0.1609944232617458, 0.23948565036486213],
        [
            0.5193110408863679,
            0.4573045936607871,
            -0.5459190446555137,
            -0.47239917081689103,
        ],
    ),
)

publish_pose(end_pose.pose, "end pose") # publish the pose 

# Get stable poses
object_pose = Pose(
    [0.8906544514627149, -0.02178125628221769, 0.23297247505313398],
    [-3.134202480316162, -0.009103048592805862, 1.554972529411316],
)
publish_pose(object_pose, "manipulation pose") # publish the object pose 

SPG.start_calculation("puzzle_trapezoid", object_pose, 1)
return_code, object_with_poses = SPG.get_poses()

# Generate valid pick poses for stable poses
picks_list = []

for object_with_pose in object_with_poses:
    DP.start_detection_with_known_pose(
        object_with_pose, workspace_name, gripper_name
    ) # getting up the database client with detection with known pose 
    return_code, picks = DP.get_picks()
    publish_pose(object_with_pose.pose, "stable pose")
    picks_list.append(picks)

print("start:")
for pick in picks:
    print(pick.__dict__)
DP.start_detection_with_known_pose(end_pose, workspace_name, gripper_name) # getting up the start detection with the known pose
return_code, end_picks = DP.get_picks()
print("end:")
for pick in end_picks:
    print(pick.__dict__)

OM = ObjectManipulationClient() # setting up the Object Manipulation client 
return_code, sequence = OM.get_manipulation_sequence(
    start_picks,
    end_picks,
    valid_picks=picks_list,
    prefered_orientation=np.array([0.0, 0.0, -1.0]),
)

for step in sequence.manipulation_steps:

    publish_pose(step.pick.object_with_pose.pose, "pick pose") # publishing the pose for pick pose 
    print(step.__dict__)
    mj(step.pick.approach_sequence.pre_pose.joint_state.joint_state)
    print("!!!!!!!!")
    ml(step.pick.approach_sequence.pose.pose.to_list())
    print("!!!!!!!!")
    ml(step.pick.approach_sequence.post_pose.pose.to_list())
    publish_pose(step.place.object_with_pose.pose, "place pose")
    print("!!!!!!!!")
    mj(step.place.approach_sequence.post_pose.joint_state.joint_state)
    print("!!!!!!!!")
    ml(step.place.approach_sequence.pose.pose.to_list())
    print("!!!!!!!!")
    ml(step.place.approach_sequence.pre_pose.pose.to_list())
    print("!!!!!!!!")
"""
print(object_names)
return_code = DP.start_detection(object_names, workspace_name, bin_name, gripper_name)
# return_code = PP.start_detection(workspace_name, gripper_name)
return_code.__dict__
print("###############")
return_code, picks = DP.get_picks()
print(return_code.__dict__)
for pick in picks:
    print(pick.__dict__)
print("###############")


# pick = picks[0]
# print(pick.approach_sequence.pre_pose.joint_state.joint_state)
# print(pick.approach_sequence.pose.pose.to_list())
# print(pick.approach_sequence.post_pose.pose.to_list())
# mj(pick.approach_sequence.pre_pose.joint_state.joint_state)
# print("!!!!!!!!")
# ml(pick.approach_sequence.pose.pose.to_list())
# print("!!!!!!!!")
# ml(pick.approach_sequence.post_pose.pose.to_list())
# print("!!!!!!!!")
#PP.start_detection(workspace_name,gripper_name, object_name="abc")
"""
