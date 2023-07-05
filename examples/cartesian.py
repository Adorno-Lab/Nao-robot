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

    # There are four methods available for cartesian control of the robot. By
    # default, the script uses positionInterpolations(), a blocking method that
    # receives pose vectors as parameter. If there is a commandline definition
    # about the method to be used, it overwrites the default.
    #   method 1: positionInterpolations() - blocking call, pose vector
    #   method 2: transformInterpolations() - blocking call, HTM
    #   method 3: setPositions() - non-blocking call, pose vector
    #   method 4: setTransforms() - non-blocking call, HTM
    method = 1
    if len(sys.argv) > 2:
        method = int(sys.argv[2])

    #  ======== USING positionInterpolations() METHOD ==========================
    if method == 1:
        # ================= Making the arms move ===============================
        arms = ["LArm", "RArm"]
        for arm in arms:
            # The getPosition method returns a vector containing the pose
            # (x, y, z, wx, wy, wz) of any joint, chain, or sensor in meters and
            # radians, relative to the chosen frame. The last parameter sets if
            # the sensors will be used to determine the pose.
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
            t_x = 1
            t_y = 2
            t_z = 4
            r_wx = 8
            r_wy = 16
            r_wz = 32
            # For example, if you want to control only position (x, y, z), the
            # parameter should be t_x + t_y + t_z = 7.
            # The positionInterpolations method is a blocking call.
            motion_proxy.positionInterpolations(arm,  # effector
                                                motion.FRAME_ROBOT,  # frame
                                                [target, initial_pose],  # target vector
                                                t_x + t_y + t_z,  # what to control
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
                                            r_wx + r_wy + r_wz,  # what to control
                                            [3, 6, 9, 12])  # relative times in seconds corresponding to the path points

    #  ======== USING transformInterpolations() METHOD =========================
    if method == 2:
        # ================= Making the arms move ===============================
        arms = ["LArm", "RArm"]
        for arm in arms:
            # The getTransform method returns an homogeneous transformation
            # matrix (HTM) to any joint, chain, or sensor relative to the chosen
            # frame. The last parameter sets if the sensors will be used to
            # determine the transformation.
            initial_transform = motion_proxy.getTransform(arm,
                                                          motion.FRAME_ROBOT,
                                                          False)

            t_x = 1
            t_y = 2
            t_z = 4
            r_wx = 8
            r_wy = 16
            r_wz = 32

            # Defining the HTM with different translation only:
            if arm == "LArm":
                target = [initial_transform[0], initial_transform[1], initial_transform[2], 0.06,
                          initial_transform[4], initial_transform[5], initial_transform[6], 0.05,
                          initial_transform[8], initial_transform[9], initial_transform[10], 0.3,
                          0, 0, 0, 1]
            else:
                target = [initial_transform[0], initial_transform[1], initial_transform[2], 0.06,
                          initial_transform[4], initial_transform[5], initial_transform[6], -0.05,
                          initial_transform[8], initial_transform[9], initial_transform[10], 0.3,
                          0, 0, 0, 1]

            # The transformInterpolations method is a blocking call.
            motion_proxy.transformInterpolations(arm,  # effector
                                                 motion.FRAME_ROBOT,  # frame
                                                 [target, initial_transform],  # target vector
                                                 t_x + t_y + t_z,  # what to control
                                                 [3, 6])  # relative times in seconds corresponding to the path points

        # ================= Making the head move ===============================
        initial_transform = motion_proxy.getTransform("Head",
                                                      motion.FRAME_ROBOT, False)

        # Defining the HTMs with different rotations only:
        angle = 30 * math.pi / 180
        target1 = [math.cos(angle), -math.sin(angle), 0, initial_transform[3],
                   math.sin(angle), math.cos(angle), 0, initial_transform[7],
                   0, 0, 1, initial_transform[11],
                   0, 0, 0, 1]
        target2 = [math.cos(-angle), -math.sin(-angle), 0, initial_transform[3],
                   math.sin(-angle), math.cos(-angle), 0, initial_transform[7],
                   0, 0, 1, initial_transform[11],
                   0, 0, 0, 1]
        target = [target1, initial_transform, target2, initial_transform]

        motion_proxy.transformInterpolations("Head",  # effector
                                             motion.FRAME_ROBOT,  # frame
                                             target,  # target vector
                                             r_wx + r_wy + r_wz,  # what to control
                                             [3, 6, 9, 12])  # relative times in seconds corresponding to the path points

    #  ======== USING setPositions() METHOD ====================================
    if method == 3:
        # ================= Making the arms move ===============================
        arms = ["LArm", "RArm"]
        for arm in arms:
            initial_pose = motion_proxy.getPosition(arm,
                                                    motion.FRAME_ROBOT, False)

            if arm == "LArm":
                target = [0.06, 0.05, 0.3,
                          initial_pose[3], initial_pose[4], initial_pose[5]]
            else:
                target = [0.06, -0.05, 0.3,
                          initial_pose[3], initial_pose[4], initial_pose[5]]

            t_x = 1
            t_y = 2
            t_z = 4
            r_wx = 8
            r_wy = 16
            r_wz = 32

            # The setPositions method is a non-blocking call.
            motion_proxy.setPositions(arm,  # effector
                                      motion.FRAME_ROBOT,  # frame
                                      target,  # target vector
                                      0.25,  # fraction of maximum speed to use
                                      t_x + t_y + t_z)  # what to control
            time.sleep(3)
            motion_proxy.setPositions(arm,  # effector
                                      motion.FRAME_ROBOT,  # frame
                                      initial_pose,  # target vector
                                      0.25,  # fraction of maximum speed to use
                                      t_x + t_y + t_z)  # what to control
            time.sleep(3)

        # ================= Making the head move ===============================
        initial_pose = motion_proxy.getPosition("Head",
                                                motion.FRAME_ROBOT, False)

        targets = [[initial_pose[0], initial_pose[1], initial_pose[2],
                    0, 0, 30 * math.pi / 180],
                   [initial_pose[0], initial_pose[1], initial_pose[2], 0, 0, 0],
                   [initial_pose[0], initial_pose[1], initial_pose[2],
                    0, 0, -30 * math.pi / 180],
                   [initial_pose[0], initial_pose[1], initial_pose[2], 0, 0, 0]]

        for target in targets:
            motion_proxy.setPositions("Head",  # effector
                                      motion.FRAME_ROBOT,  # frame
                                      target,  # target vector
                                      0.25,  # fraction of maximum speed to use
                                      r_wx + r_wy + r_wz)  # what to control
            time.sleep(3)

    #  ======== USING setTransforms() METHOD ===================================
    if method == 4:
        # ================= Making the arms move ===============================
        arms = ["LArm", "RArm"]
        for arm in arms:
            # The getTransform method returns an homogeneous transformation
            # matrix (HTM) to any joint, chain, or sensor relative to the chosen
            # frame. The last parameter sets if the sensors will be used to
            # determine the transformation.
            initial_transform = motion_proxy.getTransform(arm,
                                                          motion.FRAME_ROBOT,
                                                          False)

            t_x = 1
            t_y = 2
            t_z = 4
            r_wx = 8
            r_wy = 16
            r_wz = 32

            # Defining the HTM with different translation only:
            if arm == "LArm":
                target = [initial_transform[0], initial_transform[1],
                          initial_transform[2], 0.06,
                          initial_transform[4], initial_transform[5],
                          initial_transform[6], 0.05,
                          initial_transform[8], initial_transform[9],
                          initial_transform[10], 0.3,
                          0, 0, 0, 1]
            else:
                target = [initial_transform[0], initial_transform[1],
                          initial_transform[2], 0.06,
                          initial_transform[4], initial_transform[5],
                          initial_transform[6], -0.05,
                          initial_transform[8], initial_transform[9],
                          initial_transform[10], 0.3,
                          0, 0, 0, 1]

            # The setTransforms method is a non-blocking call.
            motion_proxy.setTransforms(arm,  # effector
                                       motion.FRAME_ROBOT,  # frame
                                       target,  # target vector
                                       0.5,  # fraction of maximum speed to use
                                       t_x + t_y + t_z)  # what to control
            time.sleep(3)
            motion_proxy.setTransforms(arm,  # effector
                                       motion.FRAME_ROBOT,  # frame
                                       initial_transform,  # target vector
                                       0.25,  # fraction of maximum speed to use
                                       t_x + t_y + t_z)  # what to control
            time.sleep(3)

        # ================= Making the head move ===============================
        initial_transform = motion_proxy.getTransform("Head",
                                                      motion.FRAME_ROBOT, False)

        # Defining the HTMs with different rotations only:
        angle = 30 * math.pi / 180
        target1 = [math.cos(angle), -math.sin(angle), 0, initial_transform[3],
                   math.sin(angle), math.cos(angle), 0, initial_transform[7],
                   0, 0, 1, initial_transform[11],
                   0, 0, 0, 1]
        target2 = [math.cos(-angle), -math.sin(-angle), 0, initial_transform[3],
                   math.sin(-angle), math.cos(-angle), 0, initial_transform[7],
                   0, 0, 1, initial_transform[11],
                   0, 0, 0, 1]
        targets = [target1, initial_transform, target2, initial_transform]

        for target in targets:
            motion_proxy.setTransforms("Head",  # effector
                                       motion.FRAME_ROBOT,  # frame
                                       target,  # target vector
                                       0.25,  # fraction of maximum speed to use
                                       r_wx + r_wy + r_wz)  # what to control
            time.sleep(3)

    # The rest() method sends the robot to a relaxed and safe position and sets
    # its motors off. For NAO H25, if the robot is standing, it goes to the
    # Crouch posture and it sets the Stiffness off. It is a blocking call.
    motion_proxy.rest()


if __name__ == "__main__":
    main()