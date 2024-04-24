from math import pi, cos, sin
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH


class NaoRobot:
    def __init__(self):
        # Robot length parameters
        self.neck_offset_z = 0.1265
        self.bottom_camera_x = 0.05071
        self.bottom_camera_z = 0.01774
        self.top_camera_x = 0.05871
        self.top_camera_z = 0.06364
        self.top_camera_wy = 0.0209
        self.bottom_camera_wy = 0.6929

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
        self.tibia_length = 0.1029
        self.foot_height = 0.04519

        # Parameters to indicate if joint is rotational or prismatic
        # (to be used with DQ_SerialManipulatorDH)
        self.dq_joint_rotational = 0
        self.dq_joint_prismatic = 1


class TopCamera(NaoRobot):
    """
    Class for the serial head chain, from torso to top camera.
    Joints: HeadYaw, HeadPitch
    """
    def get_joints_names(self) -> [str]:
        """
        Return the name of the joints in the chain.

        :return: a list with the name of the joints in the chain
        """
        return ["HeadYaw", "HeadPitch"]

    def kinematics(self) -> DQ_SerialManipulatorDH:
        """
        Create the DQ_kinematics object for the chain.

        :return: DQ_SerialManipulatorDH object
        """

        # Denavit-Hartenberg parameters
        DH_theta = [0, 0]
        DH_d = [0, 0]
        DH_a = [0, 0]
        DH_alpha = [-pi/2, 0]
        DH_type = [self.dq_joint_rotational] * 2
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tz = 1 + DQ.E * (1/2) * DQ([0, 0, self.neck_offset_z])
        obj.set_base_frame(tz)
        obj.set_reference_frame(tz)

        # Setting end effector
        rx = cos((pi/2)/2) + DQ.i * sin((pi/2)/2)
        txz = 1 + DQ.E * (1/2) * DQ([self.top_camera_x, 0, self.top_camera_z])
        ry = cos(self.top_camera_wy/2) + DQ.j * sin(self.top_camera_wy/2)
        obj.set_effector(rx * txz * ry)

        return obj


class BottomCamera(NaoRobot):
    """
    Class for the serial head chain, from torso to bottom camera.
    Joints: HeadYaw, HeadPitch
    """
    def get_joints_names(self) -> [str]:
        """
        Return the name of the joints in the chain.

        :return: a list with the name of the joints in the chain
        """
        return ["HeadYaw", "HeadPitch"]

    def kinematics(self) -> DQ_SerialManipulatorDH:
        """
        Create the DQ_kinematics object for the chain.

        :return: DQ_SerialManipulatorDH object
        """

        # Denavit-Hartenberg parameters
        DH_theta = [0, 0]
        DH_d = [0, 0]
        DH_a = [0, 0]
        DH_alpha = [-pi/2, 0]
        DH_type = [self.dq_joint_rotational] * 2
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tz = 1 + DQ.E * (1/2) * DQ([0, 0, self.neck_offset_z])
        obj.set_base_frame(tz)
        obj.set_reference_frame(tz)

        # Setting end effector
        rx = cos((pi/2)/2) + DQ.i * sin((pi/2)/2)
        txz = 1 + DQ.E * (1/2) * DQ([self.bottom_camera_x, 0, self.bottom_camera_z])
        ry = cos(self.bottom_camera_wy/2) + DQ.j * sin(self.bottom_camera_wy/2)
        obj.set_effector(rx * txz * ry)

        return obj


class LeftArm(NaoRobot):
    """
    Class for the serial left arm chain, from torso to left hand.
    Joints: LShoulderPitch, LShoulderRoll, LElbowYaw,
            LElbowRoll, LWristYaw
    """
    def get_joints_names(self) -> [str]:
        """
        Return the name of the joints in the chain.

        :return: a list with the name of the joints in the chain
        """
        return ["LShoulderPitch", "LShoulderRoll", "LElbowYaw",
                "LElbowRoll", "LWristYaw"]

    def kinematics(self) -> DQ_SerialManipulatorDH:
        """
        Create the DQ_kinematics object for the chain.

        :return: DQ_SerialManipulatorDH object
        """

        # Denavit-Hartenberg parameters
        DH_theta = [0, pi/2, 0, 0, 0]
        DH_d = [0, 0, self.upper_arm_length, 0, self.lower_arm_length]
        DH_a = [0, self.elbow_offset_y, 0, 0, 0]
        DH_alpha = [pi/2, pi/2, -pi/2, pi/2, 0]
        DH_type = [self.dq_joint_rotational] * 5
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tyz = 1 + DQ.E * (1/2) * DQ([0, self.shoulder_offset_y, self.shoulder_offset_z])
        rx = cos((-pi/2)/2) + DQ.i * sin((-pi/2)/2)
        obj.set_base_frame(tyz * rx)
        obj.set_reference_frame(tyz * rx)

        # Setting end effector
        rx = cos((-pi/2)/2) + DQ.i * sin((-pi/2)/2)
        rz = cos((-pi/2)/2) + DQ.k * sin((-pi/2)/2)
        txz = 1 + DQ.E * (1/2) * DQ([self.hand_offset_x, 0, -self.hand_offset_z])
        obj.set_effector(rx * rz * txz)

        return obj


class RightArm(NaoRobot):
    """
    Class for the serial right arm chain, from torso to right hand.
    Joints: RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw
    """
    def get_joints_names(self) -> [str]:
        """
        Return the name of the joints in the chain.

        :return: a list with the name of the joints in the chain
        """
        return ["RShoulderPitch", "RShoulderRoll", "RElbowYaw",
                "RElbowRoll", "LWristYaw"]

    def kinematics(self) -> DQ_SerialManipulatorDH:
        """
        Create the DQ_kinematics object for the chain.

        :return: DQ_SerialManipulatorDH object.
        """

        # Denavit-Hartenberg parameters
        DH_theta = [0, pi/2, 0, 0, 0]
        DH_d = [0, 0, self.upper_arm_length, 0, self.lower_arm_length]
        DH_a = [0, -self.elbow_offset_y, 0, 0, 0]
        DH_alpha = [pi/2, pi/2, -pi/2, pi/2, 0]
        DH_type = [self.dq_joint_rotational] * 5
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tyz = 1 + DQ.E * (1/2) * DQ([0, -self.shoulder_offset_y, self.shoulder_offset_z])
        rx = cos((-pi/2)/2) + DQ.i * sin((-pi/2)/2)
        obj.set_base_frame(tyz * rx)
        obj.set_reference_frame(tyz * rx)

        # Setting end effector
        rx = cos((-pi/2)/2) + DQ.i * sin((-pi/2)/2)
        rz = cos((-pi/2)/2) + DQ.k * sin((-pi/2)/2)
        txz = 1 + DQ.E * (1/2) * DQ([self.hand_offset_x, 0, -self.hand_offset_z])
        obj.set_effector(rx * rz * txz)

        return obj


class LeftLeg(NaoRobot):
    """
    Class for the serial left leg chain, from torso to left foot.
    Joints: LHipYawPitch, LHipRoll, LHipPitch,
            LKneePitch, LAnklePitch, LAnkleRoll
    """
    def get_joints_names(self) -> [str]:
        """
        Return the name of the joints in the chain.

        :return: a list with the name of the joints in the chain
        """
        return ["LHipYawPitch", "LHipRoll", "LHipPitch",
                "LKneePitch", "LAnklePitch", "LAnkleRoll"]

    def kinematics(self) -> DQ_SerialManipulatorDH:
        """
        Create the DQ_kinematics object for the chain.

        :return: DQ_SerialManipulatorDH object
        """

        # Denavit-Hartenberg parameters
        DH_theta = [pi/2, 3*pi/4, 0, 0, 0, 0]
        DH_d = [0, 0, 0, 0, 0, 0]
        DH_a = [0, 0, -self.thigh_length, -self.tibia_length, 0, 0]
        DH_alpha = [pi/2, pi/2, 0, 0, -pi/2, 0]
        DH_type = [self.dq_joint_rotational] * 6
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tyz = 1 + DQ.E * (1/2) * DQ([0, self.hip_offset_y, -self.hip_offset_z])
        rx = cos((-pi/4)/2) + DQ.i * sin((-pi/4)/2)
        obj.set_base_frame(tyz * rx)
        obj.set_reference_frame(tyz * rx)

        # Setting end effector
        ry = cos((pi/2)/2) + DQ.j * sin((pi/2)/2)
        rz = cos(pi/2) + DQ.k * sin(pi/2)
        tz = 1 + DQ.E * (1/2) * DQ([0, 0, -self.foot_height])
        obj.set_effector(ry * rz * tz)

        return obj


class RightLeg(NaoRobot):
    """
    Class for the serial right leg chain, from torso to right foot.
    Joints: RHipYawPitch, RHipRoll, RHipPitch,
            RKneePitch, RAnklePitch, RAnkleRoll
    """
    def get_joints_names(self) -> [str]:
        """
        Return the name of the joints in the chain.

        :return: a list with the name of the joints in the chain
        """
        return ["RHipYawPitch", "RHipRoll", "RHipPitch",
                "RKneePitch", "RAnklePitch", "RAnkleRoll"]

    def kinematics(self) -> DQ_SerialManipulatorDH:
        """
        Create the DQ_kinematics object for the chain.

        :return: DQ_SerialManipulatorDH object
        """

        # Denavit-Hartenberg parameters
        DH_theta = [pi/2, pi/4, 0, 0, 0, 0]
        DH_d = [0, 0, 0, 0, 0, 0]
        DH_a = [0, 0, -self.thigh_length, -self.tibia_length, 0, 0]
        DH_alpha = [pi/2, pi/2, 0, 0, -pi/2, 0]
        DH_type = [self.dq_joint_rotational] * 6
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tyz = 1 + DQ.E * (1/2) * DQ([0, -self.hip_offset_y, -self.hip_offset_z])
        rx = cos((pi/4)/2) + DQ.i * sin((pi/4)/2)
        obj.set_base_frame(tyz * rx)
        obj.set_reference_frame(tyz * rx)

        # Setting end effector
        ry = cos((pi/2)/2) + DQ.j * sin((pi/2)/2)
        rz = cos(pi/2) + DQ.k * sin(pi/2)
        tz = 1 + DQ.E * (1/2) * DQ([0, 0, -self.foot_height])
        obj.set_effector(ry * rz * tz)

        return obj

