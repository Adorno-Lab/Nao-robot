import numpy as np
from math import acos, sin
from dqrobotics import *
from NaoRobot import *


def main():
    robot = NaoRobot()
    q = np.array([0, 0])
    topcamera = robot.top_camera_kinematics()

    x_r = topcamera.fkm(q)

    print("Translation: ", x_r.translation())
    print("Rotation: ", x_r.rotation())
    print("Angle: ", 2*acos(vec3(Re(x_r.rotation()))[0]))


if __name__ == "__main__":
    main()