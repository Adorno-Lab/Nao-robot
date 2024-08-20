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

    def get_joints_limits(self, current_angle: float = 0.0) -> [float]:
        """
        Return the limits of the joints in the chain.

        :param current_angle: current HeadYaw value in radians (default: 0)
        :return: a list with the lower and upper limits of the joints
        """
        tolerance = 0.1

        # Anti collision limitation: HeadPitch limits depend on HeadYaw value
        if abs(current_angle - pi*(-119.52)/180) < tolerance:
            variable_limits = [-25.73, 18.91]
        elif abs(current_angle - pi * (-87.49) / 180) < tolerance:
            variable_limits = [-18.91, 11.46]
        elif abs(current_angle - pi * (-62.45) / 180) < tolerance:
            variable_limits = [-24.64, 17.19]
        elif abs(current_angle - pi * (-51.74) / 180) < tolerance:
            variable_limits = [-27.50, 18.91]
        elif abs(current_angle - pi * (-43.32) / 180) < tolerance:
            variable_limits = [-31.40, 21.20]
        elif abs(current_angle - pi * (-27.85) / 180) < tolerance:
            variable_limits = [-38.50, 24.18]
        elif abs(current_angle - pi * 0.0 / 180) < tolerance:
            variable_limits = [-38.50, 29.51]
        elif abs(current_angle - pi * 27.85 / 180) < tolerance:
            variable_limits = [-38.50, 24.18]
        elif abs(current_angle - pi * 43.32 / 180) < tolerance:
            variable_limits = [-31.40, 21.20]
        elif abs(current_angle - pi * 51.74 / 180) < tolerance:
            variable_limits = [-27.50, 18.91]
        elif abs(current_angle - pi * 62.45 / 180) < tolerance:
            variable_limits = [-24.64, 17.19]
        elif abs(current_angle - pi * 87.49 / 180) < tolerance:
            variable_limits = [-18.91, 11.46]
        elif abs(current_angle - pi * 119.52 / 180) < tolerance:
            variable_limits = [-25.73, 18.91]
        else:
            variable_limits = [-38.5, 29.5]

        # Lower and upper limits in degrees
        lower = [-119.5,  # HeadYaw
                 variable_limits[0]]  # HeadPitch
        upper = [119.5,  # HeadYaw
                 variable_limits[1]]  # HeadPitch
        limits = lower + upper

        # Converting to radians
        limits = [pi * x / 180 for x in limits]

        return limits

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
        DH_type = [self.dq_joint_rotational]*2
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tz = 1 + DQ.E*0.5*DQ([0, 0, self.neck_offset_z])
        obj.set_base_frame(tz)
        obj.set_reference_frame(tz)

        # Setting end effector
        rx = cos((pi/2)/2) + DQ.i*sin((pi/2)/2)
        txz = 1 + DQ.E*0.5*DQ([self.top_camera_x, 0, self.top_camera_z])
        ry = cos(self.top_camera_wy/2) + DQ.j*sin(self.top_camera_wy/2)
        obj.set_effector(rx*txz*ry)

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

    def get_joints_limits(self, current_angle: float = 0.0) -> [float]:
        """
        Return the limits of the joints in the chain.

        :param current_angle: current HeadYaw value in radians (default: 0)
        :return: a list with the lower and upper limits of the joints
        """
        tolerance = 0.1

        # Anti collision limitation: HeadPitch limits depend on HeadYaw value
        if abs(current_angle - pi * (-119.52) / 180) < tolerance:
            variable_limits = [-25.73, 18.91]
        elif abs(current_angle - pi * (-87.49) / 180) < tolerance:
            variable_limits = [-18.91, 11.46]
        elif abs(current_angle - pi * (-62.45) / 180) < tolerance:
            variable_limits = [-24.64, 17.19]
        elif abs(current_angle - pi * (-51.74) / 180) < tolerance:
            variable_limits = [-27.50, 18.91]
        elif abs(current_angle - pi * (-43.32) / 180) < tolerance:
            variable_limits = [-31.40, 21.20]
        elif abs(current_angle - pi * (-27.85) / 180) < tolerance:
            variable_limits = [-38.50, 24.18]
        elif abs(current_angle - pi * 0.0 / 180) < tolerance:
            variable_limits = [-38.50, 29.51]
        elif abs(current_angle - pi * 27.85 / 180) < tolerance:
            variable_limits = [-38.50, 24.18]
        elif abs(current_angle - pi * 43.32 / 180) < tolerance:
            variable_limits = [-31.40, 21.20]
        elif abs(current_angle - pi * 51.74 / 180) < tolerance:
            variable_limits = [-27.50, 18.91]
        elif abs(current_angle - pi * 62.45 / 180) < tolerance:
            variable_limits = [-24.64, 17.19]
        elif abs(current_angle - pi * 87.49 / 180) < tolerance:
            variable_limits = [-18.91, 11.46]
        elif abs(current_angle - pi * 119.52 / 180) < tolerance:
            variable_limits = [-25.73, 18.91]
        else:
            variable_limits = [-38.5, 29.5]

        # Lower and upper limits in degrees
        lower = [-119.5,  # HeadYaw
                 variable_limits[0]]  # HeadPitch
        upper = [119.5,  # HeadYaw
                 variable_limits[1]]  # HeadPitch
        limits = lower + upper

        # Converting to radians
        limits = [pi * x / 180 for x in limits]

        return limits

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
        DH_type = [self.dq_joint_rotational]*2
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tz = 1 + DQ.E*0.5*DQ([0, 0, self.neck_offset_z])
        obj.set_base_frame(tz)
        obj.set_reference_frame(tz)

        # Setting end effector
        rx = cos((pi/2)/2) + DQ.i*sin((pi/2)/2)
        txz = 1 + DQ.E*0.5*DQ([self.bottom_camera_x, 0, self.bottom_camera_z])
        ry = cos(self.bottom_camera_wy/2) + DQ.j*sin(self.bottom_camera_wy/2)
        obj.set_effector(rx*txz*ry)

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

    def get_joints_limits(self) -> [float]:
        """
        Return the limits of the joints in the chain.

        :return: a list with the lower and upper limits of the joints
        """
        # Lower and upper limits in degrees
        lower = [-119.5,  # LShoulderPitch
                 -18,  # LShoulderRoll
                 -119.5,  # LElbowYaw
                 -88.5,  # LElbowRoll
                 -104.5]  # LWristYaw
        upper = [119.5,  # LShoulderPitch
                 76,  # LShoulderRoll
                 119.5,  # LElbowYaw
                 -2,  # LElbowRoll
                 104.5]  # LWristYaw
        limits = lower + upper

        # Converting to radians
        limits = [pi * x / 180 for x in limits]

        return limits

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
        DH_type = [self.dq_joint_rotational]*5
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tyz = 1 + DQ.E*0.5*DQ( [0,
                                self.shoulder_offset_y,
                                self.shoulder_offset_z])
        rx = cos((-pi/2)/2) + DQ.i*sin((-pi/2)/2)
        obj.set_base_frame(tyz*rx)
        obj.set_reference_frame(tyz*rx)

        # Setting end effector
        rx = cos((-pi/2)/2) + DQ.i*sin((-pi/2)/2)
        rz = cos((-pi/2)/2) + DQ.k*sin((-pi/2)/2)
        txz = 1 + DQ.E*0.5*DQ([self.hand_offset_x, 0, -self.hand_offset_z])
        obj.set_effector(rx*rz*txz)

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

    def get_joints_limits(self) -> [float]:
        """
        Return the limits of the joints in the chain.

        :return: a list with the lower and upper limits of the joints
        """
        # Lower and upper limits in degrees
        lower = [-119.5,  # RShoulderPitch
                 -76,  # RShoulderRoll
                 -119.5,  # RElbowYaw
                 2,  # RElbowRoll
                 -104.5]  # RWristYaw
        upper = [119.5,  # RShoulderPitch
                 18,  # RShoulderRoll
                 119.5,  # RElbowYaw
                 88.5,  # RElbowRoll
                 104.5]  # RWristYaw
        limits = lower + upper

        # Converting to radians
        limits = [pi * x / 180 for x in limits]

        return limits

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
        DH_type = [self.dq_joint_rotational]*5
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tyz = 1 + DQ.E*0.5*DQ([0,
                               -self.shoulder_offset_y,
                               self.shoulder_offset_z])
        rx = cos((-pi/2)/2) + DQ.i*sin((-pi/2)/2)
        obj.set_base_frame(tyz*rx)
        obj.set_reference_frame(tyz*rx)

        # Setting end effector
        rx = cos((-pi/2)/2) + DQ.i*sin((-pi/2)/2)
        rz = cos((-pi/2)/2) + DQ.k*sin((-pi/2)/2)
        txz = 1 + DQ.E*0.5*DQ([self.hand_offset_x, 0, -self.hand_offset_z])
        obj.set_effector(rx*rz*txz)

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

    def get_joints_limits(self, current_angle: float = 0.0) -> [float]:
        """
        Return the limits of the joints in the chain.

        :param current_angle: current LAnklePitch value in radians (default: 0)
        :return: a list with the lower and upper limits of the joints
        """
        tolerance = 0.1

        # Anti collision limitation: LAnkleRoll limits depend on LAnklePitch value
        if abs(current_angle - pi * (-68.15) / 180) < tolerance:
            variable_limits = [-2.86, 4.30]
        elif abs(current_angle - pi * (-48.13) / 180) < tolerance:
            variable_limits = [-10.31, 9.74]
        elif abs(current_angle - pi * (-40.11) / 180) < tolerance:
            variable_limits = [-22.80, 12.61]
        elif abs(current_angle - pi * (-25.78) / 180) < tolerance:
            variable_limits = [-22.80, 44.06]
        elif abs(current_angle - pi * 5.73 / 180) < tolerance:
            variable_limits = [-22.80, 44.06]
        elif abs(current_angle - pi * 20.05 / 180) < tolerance:
            variable_limits = [-22.80, 31.54]
        elif abs(current_angle - pi * 52.87 / 180) < tolerance:
            variable_limits = [0.0, 2.86]
        else:
            variable_limits = [-22.79, 44.06]

        # Lower and upper limits in degrees
        lower = [-65.62,  # LHipYawPitch
                 -21.74,  # LHipRoll
                 -88,  # LHipPitch
                 -5.29,  # LKneePitch
                 -68.15,  # LAnklePitch
                 variable_limits[0]]  # LAnkleRoll
        upper = [42.44,  # LHipYawPitch
                 45.29,  # LHipRoll
                 27.73,  # LHipPitch
                 121.04,  # LKneePitch
                 52.86,  # LAnklePitch
                 variable_limits[1]]  # LAnkleRoll
        limits = lower + upper

        # Converting to radians
        limits = [pi * x / 180 for x in limits]

        return limits

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
        DH_type = [self.dq_joint_rotational]*6
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tyz = 1 + DQ.E*0.5*DQ([0, self.hip_offset_y, -self.hip_offset_z])
        rx = cos((-pi/4)/2) + DQ.i*sin((-pi/4)/2)
        obj.set_base_frame(tyz*rx)
        obj.set_reference_frame(tyz*rx)

        # Setting end effector
        ry = cos((pi/2)/2) + DQ.j*sin((pi/2)/2)
        rz = cos(pi/2) + DQ.k*sin(pi/2)
        tz = 1 + DQ.E*0.5*DQ([0, 0, -self.foot_height])
        obj.set_effector(ry*rz*tz)

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

    def get_joints_limits(self, current_angle: float = 0.0) -> [float]:
        """
        Return the limits of the joints in the chain.

        :param current_angle: current RAnklePitch value in radians (default: 0)
        :return: a list with the lower and upper limits of the joints
        """
        tolerance = 0.1

        # Anti collision limitation: RAnkleRoll limits depend on RAnklePitch value
        if abs(current_angle - pi * (-68.15) / 180) < tolerance:
            variable_limits = [-4.30, 2.86]
        elif abs(current_angle - pi * (-48.13) / 180) < tolerance:
            variable_limits = [-9.74, 10.31]
        elif abs(current_angle - pi * (-40.11) / 180) < tolerance:
            variable_limits = [-12.61, 22.80]
        elif abs(current_angle - pi * (-25.78) / 180) < tolerance:
            variable_limits = [-44.06, 22.80]
        elif abs(current_angle - pi * 5.73 / 180) < tolerance:
            variable_limits = [-44.06, 22.80]
        elif abs(current_angle - pi * 20.05 / 180) < tolerance:
            variable_limits = [-31.54, 22.80]
        elif abs(current_angle - pi * 52.87 / 180) < tolerance:
            variable_limits = [2.86, 0.0]
        else:
            variable_limits = [-44.06, 22.80]

        # Lower and upper limits in degrees
        lower = [-65.62,  # RHipYawPitch
                 -45.29,  # RHipRoll
                 -88,  # RHipPitch
                 -5.90,  # RKneePitch
                 -67.97,  # RAnklePitch
                 variable_limits[0]]  # RAnkleRoll
        upper = [42.44,  # RHipYawPitch
                 21.74,  # RHipRoll
                 27.73,  # RHipPitch
                 121.47,  # RKneePitch
                 53.40,  # RAnklePitch
                 variable_limits[1]]  # RAnkleRoll
        limits = lower + upper

        # Converting to radians
        limits = [pi * x / 180 for x in limits]

        return limits

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
        DH_type = [self.dq_joint_rotational]*6
        DH_matrix = [DH_theta, DH_d, DH_a, DH_alpha, DH_type]

        obj = DQ_SerialManipulatorDH(DH_matrix)

        # Setting base and reference to the torso
        tyz = 1 + DQ.E*0.5*DQ([0, -self.hip_offset_y, -self.hip_offset_z])
        rx = cos((pi/4)/2) + DQ.i*sin((pi/4)/2)
        obj.set_base_frame(tyz*rx)
        obj.set_reference_frame(tyz*rx)

        # Setting end effector
        ry = cos((pi/2)/2) + DQ.j*sin((pi/2)/2)
        rz = cos(pi/2) + DQ.k*sin(pi/2)
        tz = 1 + DQ.E*0.5*DQ([0, 0, -self.foot_height])
        obj.set_effector(ry*rz*tz)

        return obj
