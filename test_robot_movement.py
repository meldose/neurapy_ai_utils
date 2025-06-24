from neurapy_ai.clients.database_client import DatabaseClient # importing the database client from neurapy.clients

from neurapy_ai_utils.robot.maira_kinematics import MairaKinematics # importing the kinematics from neurapy.utils


# defining the database client 
db_client = DatabaseClient() 
robot = MairaKinematics(require_elbow_up=False)

# setting up the points
point1_name = "P9"
point2_name = "P10"
point3_name = "P11"

res, joint_poses_1 = db_client.get_joint_positions(point1_name) # getting the joint positions in point1 
res, point_1 = db_client.get_pose(point1_name) # getting the pose of the point1


res, joint_poses_2 = db_client.get_joint_positions(point2_name) # getting the joint position for point2
res, point_2 = db_client.get_pose(point2_name) # getting the pose for the point 2 

res, joint_poses_3 = db_client.get_joint_positions(point3_name) # getting the joint positions for point 3
res, point_3 = db_client.get_pose(point3_name) # getting the joint pose for point 3

#setting the points 1,2 and 3 as a list 
joint_poses_1 = list(joint_poses_1)
joint_poses_2 = list(joint_poses_2)
joint_poses_3 = list(joint_poses_3)

point_1 = point_1.to_list()
point_2 = point_2.to_list()
point_3 = point_3.to_list()

robot.move_joint_to_joint(joint_poses_1)# moving the robot to the joint poses1
robot.move_joint_to_joint(joint_poses_2)# moving the robot to the joint poses2

robot.move_joint_to_cartesian(point_1) # moving the robot to the point1
robot.move_joint_to_cartesian(point_2) # moving the robot to the point2


robot.move_linear(point_1) # moving the robot to the point1
robot.move_linear(point_2) # moving the robot to the point2
robot.move_linear(point_3) # moving the robot to the point3

robot.move_linear_via_points([point_1, point_2, point_3], blending_radius=0.1) # moving to move_linear_through_points
robot.move_joint_via_points([joint_poses_1, joint_poses_2, joint_poses_3]) # moving to move_joint_via_points

robot.finish() # stopping the robot movement
