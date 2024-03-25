from math import pi, cos, sin
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH


class NaoRobot:
    """
    Class to deal with the Aldebaran's robot NAO H25 (v5).

    It uses the Denavit Hartenberg model provided by

    """
    def __init__(self):
        # Robot length parameters.
        self.neck_offset_z = 0.1265
        self.bottom_camera_x = 0.05071
        self.bottom_camera_z = 0.01774
        self.top_camera_x = 0.05871
        self.top_camera_z = 0.06364
        self.top_camera_angle_y = 1.2 * pi / 180
        self.bottom_camera_angle_y = 39.7 * pi / 180

        self.shoulder_offset_z = 0.100
        self.shoulder_offset_y = 0.098
        self.elbow_offset_y = 0.015
        self.upper_arm_length = 0.105
        self.lower_arm_length = 0.05595
        self.hand_offset_x = 0.05775
        self.hand_offset_z = 0.01231

        self.hip_offset_z = 0.085
        self.hip_offset_y = 0.050
        self.thigh_length = 0.100
        self.tibia_length = 0.10290
        self.foot_height = 0.04519

        # Parameters to indicate if joint is rotational or prismatic,
        # to be used with DQ_SerialManipulatorDH.
        self.dq_joint_rotational = 0
        self.dq_joint_prismatic = 1


class TopCamera(NaoRobot):
    """
    Class for the serial head chain, from torso to top camera.
    Joints: HeadYaw, HeadPitch
    """
    def kinematics(self):
        """
        Create the DQ_kinematics object for the chain. It uses the
        Denavit-Hartenberg parameters provided by
        https://www.cs.umd.edu/~nkofinas/Projects/KofinasThesis.pdf.

        :return: DQ_SerialManipulatorDH object with kinematics info.
        """
        DH_theta = [0, 0]
        DH_d = [self.neck_offset_z, 0]
        DH_a = [0, 0]
        DH_alpha = [0, -pi/2]
        DH_type = [self.dq_joint_rotational] * 2
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting the end effector:
        txz = 1 + DQ.E * (1/2) * DQ([self.top_camera_x, 0, self.top_camera_z])
        rx = cos((pi/2)/2) + DQ.i * sin((pi/2)/2)
        ry = cos(self.top_camera_angle_y/2) + DQ.j * sin(self.top_camera_angle_y/2)
        x_ee = rx * txz * ry
        obj.set_effector(x_ee)

        return obj


class BottomCamera(NaoRobot):
    """
    Class for the serial head chain, from torso to bottom camera.
    Joints: HeadYaw, HeadPitch
    """
    def kinematics(self):
        """
        Create the DQ_kinematics object for the chain. It uses the
        Denavit-Hartenberg parameters provided by
        https://www.cs.umd.edu/~nkofinas/Projects/KofinasThesis.pdf.

        :return: DQ_SerialManipulatorDH object with kinematics info.
        """
        DH_theta = [0, 0]
        DH_d = [self.neck_offset_z, 0]
        DH_a = [0, 0]
        DH_alpha = [0, -pi/2]
        DH_type = [self.dq_joint_rotational] * 2
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting the end effector:
        txz = 1 + DQ.E * (1/2) * DQ([self.bottom_camera_x, 0, self.bottom_camera_z])
        rx = cos((pi/2)/2) + DQ.i * sin((pi/2)/2)
        ry = cos(self.bottom_camera_angle_y/2) + DQ.j * sin(self.bottom_camera_angle_y/2)
        x_ee = rx * txz * ry
        obj.set_effector(x_ee)

        return obj


class LeftArm(NaoRobot):
    """
    Class for the serial left arm chain, from torso to left hand.
    Joints: LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRow
    """
    def kinematics(self):
        """
        Create the DQ_kinematics object for the chain. It uses the
        Denavit-Hartenberg parameters provided by
        https://www.cs.umd.edu/~nkofinas/Projects/KofinasThesis.pdf.

        :return: DQ_SerialManipulatorDH object with kinematics info.
        """

        DH_theta = [0, 0, pi/2, 0, 0]
        DH_d = [self.shoulder_offset_z, self.shoulder_offset_y, 0, self.upper_arm_length, 0]
        DH_a = [0, 0, self.elbow_offset_y, 0, 0]
        DH_alpha = [-pi/2, pi/2, pi/2, -pi/2, pi/2]
        DH_type = [self.dq_joint_rotational] * 5
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting the end effector:
        txz = 1 + DQ.E * (1/2) * DQ([self.hand_offset_x + self.lower_arm_length, 0, -self.hand_offset_z])
        rx = cos((-pi/2)/2) + DQ.i * sin((-pi/2)/2)
        rz = cos((-pi/2)/2) + DQ.k * sin((-pi/2)/2)
        x_ee = rx * rz * txz
        obj.set_effector(x_ee)

        return obj


class RightArm(NaoRobot):
    """
    Class for the serial right arm chain, from torso to right hand.
    Joints: RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRow
    """
    def kinematics(self):
        """
        Create the DQ_kinematics object for the chain. It uses the
        Denavit-Hartenberg parameters provided by
        https://www.cs.umd.edu/~nkofinas/Projects/KofinasThesis.pdf.

        :return: DQ_SerialManipulatorDH object with kinematics info.
        """
        DH_theta = [0, 0, pi/2, 0, 0]
        DH_d = [self.shoulder_offset_z, -self.shoulder_offset_y, 0, self.upper_arm_length, 0]
        DH_a = [0, 0, -self.elbow_offset_y, 0, 0]
        DH_alpha = [-pi/2, pi/2, pi/2, -pi/2, pi/2]
        DH_type = [self.dq_joint_rotational] * 5
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting the end effector:
        txz = 1 + DQ.E * (1/2) * DQ([self.lower_arm_length, 0, 0])
        rx = cos((-pi/2)/2) + DQ.i * sin((-pi/2)/2)
        rz = cos((-pi/2)/2) + DQ.k * sin((-pi/2)/2)
        x_ee = rx * rz * txz
        obj.set_effector(x_ee)

        return obj


class LeftLeg(NaoRobot):
    """
    Class for the serial left leg chain, from torso to left foot.
    Joints: LHipYawPitch, LHipRoll, LHipPitch,
            LKneePitch, LAnklePitch, LAnkleRoll
    """
    def kinematics(self):
        """
        Create the DQ_kinematics object for the chain. It uses the
        Denavit-Hartenberg parameters provided by
        https://www.cs.umd.edu/~nkofinas/Projects/KofinasThesis.pdf.

        :return: DQ_SerialManipulatorDH object with kinematics info.
        """
        DH_theta = [-pi/2, pi/4, 0, 0, 0, 0]
        DH_d = [0, 0, 0, 0, 0, 0]
        DH_a = [0, 0, 0, -self.thigh_length, -self.tibia_length, 0]
        DH_alpha = [-3*pi/4, -pi/2, pi/2, 0, 0, -pi/2]
        DH_type = [self.dq_joint_rotational] * 6
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting the base:
        x_base = 1 + DQ.E * (1/2) * DQ([0,
                                        self.hip_offset_y,
                                        -self.hip_offset_z])
        obj.set_reference_frame(x_base)

        # Setting the end effector:
        t_ee = 1 + DQ.E * (1/2) * DQ([0, 0, -self.foot_height])
        angle = pi
        axis = DQ.k
        r_ee_1 = cos(angle/2) + axis * sin(angle/2)
        angle = -pi/2
        axis = DQ.j
        r_ee_2 = cos(angle/2) + axis * sin(angle/2)
        x_ee = r_ee_1 * r_ee_2 * t_ee
        obj.set_effector(x_ee)

        return obj


class RightLeg(NaoRobot):
    """
    Class for the serial right leg chain, from torso to right foot.
    Joints: RHipYawPitch, RHipRoll, RHipPitch,
            RKneePitch, RAnklePitch, RAnkleRoll
    """
    def kinematics(self):
        """
        Create the DQ_kinematics object for the chain. It uses the
        Denavit-Hartenberg parameters provided by
        https://www.cs.umd.edu/~nkofinas/Projects/KofinasThesis.pdf.

        :return: DQ_SerialManipulatorDH object with kinematics info.
        """
        DH_theta = [-pi/2, -pi/4, 0, 0, 0, 0]
        DH_d = [0, 0, 0, 0, 0, 0]
        DH_a = [0, 0, 0, -self.thigh_length, -self.tibia_length, 0]
        DH_alpha = [-pi/4, -pi/2, pi/2, 0, 0, -pi/2]
        DH_type = [self.dq_joint_rotational] * 6
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting the base:
        x_base = 1 + DQ.E * (1/2) * DQ([0,
                                        -self.hip_offset_y,
                                        -self.hip_offset_z])
        obj.set_reference_frame(x_base)

        # Setting the end effector:
        t_ee = 1 + DQ.E * (1/2) * DQ([0, 0, -self.foot_height])
        angle = pi
        axis = DQ.k
        r_ee_1 = cos(angle/2) + axis * sin(angle/2)
        angle = -pi/2
        axis = DQ.j
        r_ee_2 = cos(angle/2) + axis * sin(angle/2)
        x_ee = r_ee_1 * r_ee_2 * t_ee
        obj.set_effector(x_ee)

        return obj

