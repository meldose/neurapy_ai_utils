from neurapy_ai.clients.database_client import DatabaseClient

from neurapy_ai_utils.robot.maira_kinematics import MairaKinematics


# defining the database client 
db_client = DatabaseClient()
robot = MairaKinematics(require_elbow_up=False)

# setting up the points
point1_name = "P9"
point2_name = "P10"
point3_name = "P11"

res, joint_poses_1 = db_client.get_joint_positions(point1_name)
res, point_1 = db_client.get_pose(point1_name)


res, joint_poses_2 = db_client.get_joint_positions(point2_name)
res, point_2 = db_client.get_pose(point2_name)

res, joint_poses_3 = db_client.get_joint_positions(point3_name)
res, point_3 = db_client.get_pose(point3_name)

joint_poses_1 = list(joint_poses_1)
joint_poses_2 = list(joint_poses_2)
joint_poses_3 = list(joint_poses_3)

point_1 = point_1.to_list()
point_2 = point_2.to_list()
point_3 = point_3.to_list()

robot.move_joint_to_joint(joint_poses_1)
robot.move_joint_to_joint(joint_poses_2)


robot.move_joint_to_cartesian(point_1)
robot.move_joint_to_cartesian(point_2)


robot.move_linear(point_1)
robot.move_linear(point_2)
robot.move_linear(point_3)

robot.move_linear_via_points([point_1, point_2, point_3], blending_radius=0.1)
robot.move_joint_via_points([joint_poses_1, joint_poses_2, joint_poses_3])

robot.finish()
