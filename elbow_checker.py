import numpy as np # imported numpy 
from neurapy_ai.utils.types import Pose
from typing import List # imported typing

# created class Elbowchecker

class ElbowChecker:
    def __init__(self, dof=7, robot_name="maira7M") -> None:
        self._dof = dof
        self._robot_name = robot_name
        if "maira" in robot_name:
            if "M" in robot_name:
                self._link3_length = 0.7
                self._link5_length = 0.7
            if "JR" in robot_name:
                self._link3_length = 0.45
                self._link5_length = 0.45
            elif "XL" in robot_name:
                self._link3_length = 0.9
                self._link5_length = 0.9
            elif "L" in robot_name:
                self._link3_length = 0.77
                self._link5_length = 0.825
            elif "S" in robot_name:
                self._link3_length = 0.525
                self._link5_length = 0.575
        elif "lara" in robot_name:
            if "8" in robot_name:
                self._link3_length = -0.6
                self._link5_length = -0.7
            elif "5" in robot_name:
                self._link3_length = -0.38
                self._link5_length = -0.42
            elif "3" in robot_name:
                self._link3_length = -0.266
                self._link5_length = -0.324
        else:
            raise ValueError(f"Elbow checker not implemented for: {robot_name}")

# created function for checking for joint state 

    def is_up(self, joint_state: List[float]) -> bool:
        """Check if elbow (i.e. axis 4) is up relative to the robot base

        Parameters
        ----------
        joint_state : List[float]
            Joint positions to check

        Returns
        -------
        bool
            True if elbow is up
        """
        if np.nan in joint_state:
            return False

# if the robot name and dof is found:
        if "maira" in self._robot_name:
            if self._dof == 7:  # for MAiRA 7
                joint2_position = joint_state[1]
                joint3_position = joint_state[2]
                joint4_position = joint_state[3]
                joint5_position = joint_state[4]
            elif self._dof == 6:  # for MAiRA 6
                joint2_position = joint_state[1]
                joint3_position = 0.0
                joint4_position = joint_state[2]
                joint5_position = joint_state[3]
            else:
                ValueError(f"ElbowChecker: Invalid DoF number: {self._dof}")

            tf_1_2 = (
                Pose.from_list([0.0, 0.0, 0.0, 0.0, 0.0, joint2_position])
                .to_numpy()
                .dot(
                    Pose.from_list(
                        [0.0, 0.0, 0.0, np.pi / 2, 0.0, 0.0]
                    ).to_numpy()
                )
            )
            tf_2_3 = (
                Pose.from_list([0.0, 0.0, 0.0, 0.0, 0.0, joint3_position])
                .to_numpy()
                .dot(
                    Pose.from_list(
                        [0.0, 0.0, self._link3_length, -np.pi / 2, 0.0, 0.0]
                    ).to_numpy()
                )
            )
            tf_3_4 = (
                Pose.from_list([0.0, 0.0, 0.0, 0.0, 0.0, joint4_position])
                .to_numpy()
                .dot(
                    Pose.from_list(
                        [0.0, 0.0, 0.0, np.pi / 2, 0.0, 0.0]
                    ).to_numpy()
                )
            )
            tf_4_5 = (
                Pose.from_list([0.0, 0.0, 0.0, 0.0, 0.0, joint5_position])
                .to_numpy()
                .dot(
                    Pose.from_list(
                        [0.0, 0.0, self._link5_length, -np.pi / 2, 0.0, 0.0]
                    ).to_numpy()
                )
            )
        elif "lara" in self._robot_name:
            joint2_position = joint_state[1]
            joint3_position = joint_state[2]
            joint4_position = joint_state[3]
            joint5_position = joint_state[4]

            tf_1_2 = (
                Pose.from_list([0.0, 0.0, 0.0, 0, 0.0, 0])
                .to_numpy()
                .dot(
                    Pose.from_list(
                        [0.0, 0.0, 0.0, joint2_position, 0.0, 0.0]
                    ).to_numpy()
                )
            )
            tf_2_3 = (
                Pose.from_list([0.0, self._link3_length, 0.0, 0.0, 0.0, 0.0])
                .to_numpy()
                .dot(
                    Pose.from_list(
                        [0.0, 0.0, 0.0, joint3_position, 0.0, 0.0]
                    ).to_numpy()
                )
            )
            tf_3_4 = (
                Pose.from_list([0.0, 0.0, 0.0, 0, 0.0, 0])
                .to_numpy()
                .dot(
                    Pose.from_list(
                        [0.0, 0.0, 0.0, 0.0, -joint4_position, 0.0]
                    ).to_numpy()
                )
            )
            tf_4_5 = (
                Pose.from_list([0.0, self._link5_length, 0.0, 0.0, 0.0, 0])
                .to_numpy()
                .dot(
                    Pose.from_list(
                        [0.0, 0.0, 0.0, joint5_position, 0.0, 0.0]
                    ).to_numpy()
                )
            )
        else:
            raise ValueError(
                f"ElbowChecker not implemented for {self._robot_name}"
            )
        
        tf_1_3 = tf_1_2.dot(tf_2_3)
        tf_1_5 = tf_1_3.dot(tf_3_4).dot(tf_4_5)
        p0 = np.zeros(3)
        p1 = tf_1_3[:3, 3]
        p2 = tf_1_5[:3, 3]
        v = (p2 - p0) / 2 - p1
        return np.sign(v[1]) == 1 # retunrn the value of the vector with sign positive 
