#!/usr/bin/env python2.7

from naoqi import ALProxy
import motion

import sys
import time
import math


def main():
    motionProxy = ALProxy("ALMotion", sys.argv[1], 9559)
    postureProxy = ALProxy("ALRobotPosture", sys.argv[1], 9559)

    # If the robot is in Crouch posture, wakeUp() command puts it in the Stand
    # posture and rest() puts it back to Crouch Posture. wakeUp and rest
    # commands do not make a difference if the robot starts sitting.
    motionProxy.wakeUp()  # wake up robot

    # Send robot to Stand Init posture:
    #postureProxy.goToPosture("Stand", 0.5)

    arms = ["LArm", "RArm"]

    for arm in arms:
        # The getPosition method returns a vector containing the pose
        # (x, y, z, wx, wy, wz) of any joint, chain, or sensor in meters and
        # radians, relative to the chosen frame (FRAME_TORSO = 0,
        # FRAME_WORLD = 1, FRAME_ROBOT = 2). The last parameter determines if
        # the sensors values will be used to determine the position.
        initial_pose = motionProxy.getPosition(arm, motion.FRAME_ROBOT, False)

        # [0.047143254429101944, 0.11501868069171906, 0.23024789988994598,
        # -0.9266910552978516, 1.0817198753356934, 0.12328142672777176]

        # Activate arm tracking (it implicitly activates Whole Body Balancer):
        motionProxy.wbEnableEffectorControl(arm, True)

        # An "Effector" is a predefined 3D point in the robot and it's generally
        # the end of a chain. The points related to "LArm" and "RArm" are inside
        # the respective hand.

        # To set the target position of the arm, the parameters are the x, y,
        # and z axis coordinates in meters relative to the FRAME_ROBOT. The
        # method wbSetEffectorControl is non blocking, so we use time.sleep to
        # allow the arm to arrive at the targe position. The recommended minimum
        # period between two successive commands is 0.2 seconds.
        if arm == "LArm":
            target = [0.15, 0.15, 0.4]
        else:
            target = [0.15, -0.15, 0.4]
        motionProxy.wbSetEffectorControl(arm, target)
        time.sleep(5)
        #motionProxy.wbSetEffectorControl(arm, initial_pose[0:3])
        #time.sleep(5)

        # Deactivate arm tracking:
        motionProxy.wbEnableEffectorControl(arm, False)

    motionProxy.rest()


if __name__ == "__main__":
    main()