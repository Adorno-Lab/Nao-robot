#!/usr/bin/env python2.7

from naoqi import ALProxy
import motion

import sys
import time
import math


def main():
    motion_proxy = ALProxy("ALMotion", sys.argv[1], 9559)
    posture_proxy = ALProxy("ALRobotPosture", sys.argv[1], 9559)
    awareness_proxy = ALProxy("ALBasicAwareness", sys.argv[1], 9559)

    # Waking up the robot sets its motors on. For NAO H25, it sets the Stiffness
    # on and keeps the current position. If it starts from the Crouch posture,
    # it makes the robot stand up; if it starts already standing or sitting, it
    # does nothing with the robot posture. It is a blocking call.
    motion_proxy.wakeUp()

    # The ALBasicAwareness module enables the robot to be aware of stimuli
    # coming from its surrounding environment.
    awareness_proxy.stopAwareness()

    # The StandInit posture is a good starting point for most of the motion
    # animations. We use the ALRobotPosture module to send the robot to
    # predefined postures such as StandInit. It is a blocking call.
    posture_proxy.goToPosture("StandInit", 0.5)

    # By default, the script uses an animation method and the
    # positionInterpolations() method. If there is a commandline definition
    # about the method, it overwrites the default.
    #   method 1: animation, positionInterpolations()
    #   method 2: animation, transformInterpolations()
    #   method 3: reactive, setPositions()
    #   method 4: reactive, setTransforms()
    method = 1
    if len(sys.argv) > 2:
        method = int(sys.argv[2])

    #  ======== USING ANIMATION METHOD - positionInterpolations ================
    if method == 1:
        # ================= Making the arms move ===============================
        arms = ["LArm", "RArm"]
        for arm in arms:
            # The getPosition method returns a vector containing the pose
            # (x, y, z, wx, wy, wz) of any joint, chain, or sensor in meters and
            # radians, relative to the chosen frame. The last parameter sets if
            # the values will be used to determine the pose.
            initial_pose = motion_proxy.getPosition(arm,
                                                    motion.FRAME_ROBOT, False)

            # To set the target position of the arm, the parameters are position
            # and orientation (x, y, z, wx, wy, wz) in meters and radians.
            if arm == "LArm":
                target = [0.06, 0.05, 0.3,
                          initial_pose[3], initial_pose[4], initial_pose[5]]
            else:
                target = [0.06, -0.05, 0.3,
                          initial_pose[3], initial_pose[4], initial_pose[5]]

            # The robot arms do not have enough degrees of freedom to enable
            # pose (position + orientation) control, so a parameter in the
            # positionInterpolations method sets what will be controlled. It is
            # an integer, obtained with the sum of numbers associated to each
            # one of the translation and rotation axes:
            pos_x = 1
            pos_y = 2
            pos_z = 4
            or_wx = 8
            or_wy = 16
            or_wz = 32
            # For example, if you want to control only position (x, y, z), the
            # parameter should be pos_x + pos_y + pos_z = 7.
            # The positionInterpolations method is a blocking call.
            motion_proxy.positionInterpolations(arm,  # effector
                                                motion.FRAME_ROBOT,  # frame
                                                [target, initial_pose],  # target vector
                                                pos_x + pos_y + pos_z,  # what to control
                                                [3, 6])  # relative times in seconds corresponding to the path points

        # ================= Making the head move ===============================
        initial_pose = motion_proxy.getPosition("Head", motion.FRAME_ROBOT, False)

        targets = [[initial_pose[0], initial_pose[1], initial_pose[2],
                    0, 0, 30 * math.pi / 180],
                   [initial_pose[0], initial_pose[1], initial_pose[2], 0, 0, 0],
                   [initial_pose[0], initial_pose[1], initial_pose[2],
                    0, 0, -30 * math.pi / 180],
                   [initial_pose[0], initial_pose[1], initial_pose[2], 0, 0, 0]]

        motion_proxy.positionInterpolations("Head",  # effector
                                            motion.FRAME_ROBOT,  # frame
                                            targets,  # target vector
                                            or_wx + or_wy + or_wz,  # what to control
                                            [3, 6, 9, 12])  # relative times in seconds corresponding to the path points

    # The rest() method sends the robot to a relaxed and safe position and sets
    # its motors off. For NAO H25, if the robot is standing, it goes to the
    # Crouch posture and it sets the Stiffness off. It is a blocking call.
    motion_proxy.rest()


if __name__ == "__main__":
    main()