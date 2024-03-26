from NaoRobot import *
from dqrobotics import *


def main():

    q = [0, 0]

    chain = TopCamera()
    x_eff = chain.kinematics().fkm(q)

    print("TRANSFORMATION GIVEN BY THE MODEL:")
    print("translation:", x_eff.translation())
    print("rotation:", x_eff.rotation())
    print("rotation angle (rad / deg):", x_eff.rotation().rotation_angle(), "/", x_eff.rotation().rotation_angle()*180/pi)
    print("rotation axis:", x_eff.rotation().rotation_axis())

    print("----------------")

    pose = [0.0587100014090538, 0.0, 0.1901399940252304, 0.0, 0.020943524315953255, 0.0]
    print("TRANSFORMATION GIVEN BY THE ROBOT:")
    print("translation:", DQ.i * pose[0] + DQ.j * pose[1] + DQ.k * pose[2])
    rx = cos(pose[3]/2) + DQ.i*sin(pose[3]/2)
    ry = cos(pose[4]/2) + DQ.j*sin(pose[4]/2)
    rz = cos(pose[5]/2) + DQ.k*sin(pose[5]/2)
    r = rz*ry*rx
    print("rotation:", rz*ry*rx)
    print("rotation angle (rad / deg):", r.rotation_angle(), "/", r.rotation_angle()*180/pi)
    print("rotation axis:", r.rotation_axis())


if __name__ == "__main__":
    main()