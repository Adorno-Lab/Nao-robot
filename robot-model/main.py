import numpy as np
from math import acos, sin
from dqrobotics import *
from NaoRobot import *


def main():
    top_camera = TopCamera()
    q = np.array([0, 0])
    x_top_camera = top_camera.kinematics().fkm(q)

    bottom_camera = BottomCamera()
    q = np.array([0, 0])
    x_bottom_camera = bottom_camera.kinematics().fkm(q)

    left_arm = LeftArm()
    q = np.array([0, 0, 0, 0])
    x_left_arm = left_arm.kinematics().fkm(q)

    right_arm = RightArm()
    q = np.array([0, 0, 0, 0])
    x_right_arm = right_arm.kinematics().fkm(q)

    left_leg = LeftLeg()
    q = np.array([0, 0, 0, 0, 0, 0])
    x_left_leg = left_leg.kinematics().fkm(q)

    right_leg = RightLeg()
    q = np.array([0, 0, 0, 0, 0, 0])
    x_right_leg = right_leg.kinematics().fkm(q)


if __name__ == "__main__":
    main()