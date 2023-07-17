#!/usr/bin/env python2.7

from naoqi import ALProxy

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

    # There are four methods available for joint control of the robot. By
    # default, the script uses angleInterpolation(), a blocking method that
    # receives absolute or relative angles and relative times as parameters. If
    # there is a commandline choice of method to be used, it overwrites the
    # default.
    #   method 1: angleInterpolation() - blocking, absolute or relative angles, relative times
    #   method 2: angleInterpolationWithSpeed() - blocking, absolute angles, speed fraction
    #   method 3: setAngles() - non-blocking, absolute angles, speed fraction
    #   method 4: changeAngles() - non-blocking, relative angles, speed fraction
    method = 1
    if len(sys.argv) > 2:
        try:
            method = int(sys.argv[2])
        except:
            method = sys.argv[2]

    #  ======== USING angleInterpolation() METHOD ==============================
    if method == 1:
        # ================= Making the arms move ===============================
        arms = ["LArm", "RArm"]
        for arm in arms:
            # The getAngles method returns the joints or chain angles. The
            # second parameter sets if the sensors will be used to determine
            # the angles.
            initial_angles = motion_proxy.getAngles(arm, False)

            # Setting the target angles of each joint in the arm chain. The
            # angle values were obtained using the getAngles method in the
            # cartesian_control.py script.
            if arm == "LArm":
                target = [[1.6154522895812988, initial_angles[0]], # LShoulderPitch
                          [0.3719860911369324, initial_angles[1]],  # LShoulderRoll
                          [-0.7981292605400085, initial_angles[2]],  # LElbowYaw
                          [-1.5378968715667725, initial_angles[3]],  # LElbowRoll
                          [-0.4528430104255676, initial_angles[4]],  # LWristYaw
                          [0.25, initial_angles[5]]]  # LHand
            else:
                target = [[1.6155904531478882, initial_angles[0]],  # RShoulderPitch
                          [-0.37190526723861694, initial_angles[1]],  # RShoulderRoll
                          [0.7983359098434448, initial_angles[2]],  # RElbowYaw
                          [1.537906289100647, initial_angles[3]],  # RElbowRoll
                          [0.45257824659347534, initial_angles[4]],  # RWristYaw
                          [0.25, initial_angles[5]]]  # RHand

            times = [[3, 6]]*len(initial_angles)

            # The angleInterpolation method is a blocking call.
            motion_proxy.angleInterpolation(arm,  # joint
                                            target,  # target angles
                                            times,  # relative times in seconds corresponding to the path points
                                            True)  # if the angles are absolute or not

        # ================= Making the head move ===============================
        initial_angle = motion_proxy.getAngles("HeadYaw", False)
        target = [30 * math.pi / 180, initial_angle[0],
                  -30 * math.pi / 180, initial_angle[0]]

        motion_proxy.angleInterpolation("HeadYaw",  # joint
                                        target,  # target angles
                                        [3, 6, 9, 12],  # relative times in seconds corresponding to the path points
                                        True)  # if the angles are absolute or not

    #  ======== USING angleInterpolationWithSpeed() METHOD =====================
    if method == 2:
        # ================= Making the arms move ===============================
        arms = ["LArm", "RArm"]
        for arm in arms:
            initial_angles = motion_proxy.getAngles(arm, False)

            if arm == "LArm":
                target = [1.6154522895812988,  # LShoulderPitch
                          0.3719860911369324,  # LShoulderRoll
                          -0.7981292605400085,  # LElbowYaw
                          -1.5378968715667725,  # LElbowRoll
                          -0.4528430104255676,  # LWristYaw
                          0.25]  # LHand
            else:
                target = [1.6155904531478882,  # RShoulderPitch
                          -0.37190526723861694,  # RShoulderRoll
                          0.7983359098434448,  # RElbowYaw
                          1.537906289100647,  # RElbowRoll
                          0.45257824659347534,  # RWristYaw
                          0.25]  # RHand

            # The angleInterpolationWithSpeed method does not accept multiple
            # target angles in one single method call, so we use a FOR loop, but
            # since it is a blocking call we do not need time.sleep().
            targets = [target, initial_angles]
            for target in targets:
                motion_proxy.angleInterpolationWithSpeed(arm,  # joint
                                                         target,  # target angles
                                                         0.1)  # fraction of maximum speed to use

        # ================= Making the head move ===============================
        initial_angle = motion_proxy.getAngles("HeadYaw", False)
        targets = [30 * math.pi / 180, initial_angle[0],
                   -30 * math.pi / 180, initial_angle[0]]

        for target in targets:
            motion_proxy.angleInterpolationWithSpeed("HeadYaw",  # joint
                                                     target,  # target angles
                                                     0.1)  # fraction of maximum speed to use

    #  ======== USING setAngles() METHOD =======================================
    if method == 3:
        # ================= Making the arms move ===============================
        arms = ["LArm", "RArm"]
        for arm in arms:
            initial_angles = motion_proxy.getAngles(arm, False)

            if arm == "LArm":
                target = [1.6154522895812988,  # LShoulderPitch
                          0.3719860911369324,  # LShoulderRoll
                          -0.7981292605400085,  # LElbowYaw
                          -1.5378968715667725,  # LElbowRoll
                          -0.4528430104255676,  # LWristYaw
                          0.25]  # LHand
            else:
                target = [1.6155904531478882,  # RShoulderPitch
                          -0.37190526723861694,  # RShoulderRoll
                          0.7983359098434448,  # RElbowYaw
                          1.537906289100647,  # RElbowRoll
                          0.45257824659347534,  # RWristYaw
                          0.25]  # RHand

            # The setAngles method is a non-blocking call.
            targets = [target, initial_angles]
            for target in targets:
                motion_proxy.setAngles(arm,  # joint
                                       target,  # target angles
                                       0.1)  # fraction of maximum speed to use
                time.sleep(2)

        # ================= Making the head move ===============================
        initial_angle = motion_proxy.getAngles("HeadYaw", False)
        targets = [30 * math.pi / 180, initial_angle[0],
                   -30 * math.pi / 180, initial_angle[0]]

        for target in targets:
            motion_proxy.setAngles("HeadYaw",  # joint
                                   target,  # target angles
                                   0.1)  # fraction of maximum speed to use
            time.sleep(2)

    #  ======== USING changeAngles() METHOD ====================================
    if method == 4:
        # ================= Making the head move ===============================
        targets = [30 * math.pi / 180, -30 * math.pi / 180,
                   -30 * math.pi / 180, 30 * math.pi / 180]

        # The changeAngles method is a non-blocking call.
        for target in targets:
            motion_proxy.changeAngles("HeadYaw",  # joint
                                      target,  # target angles
                                      0.25)  # fraction of maximum speed to use
            time.sleep(3)

    # ======== MOVING HANDS ====================================================
    if method == "hand":
        # Opening hands using the argument POST so they can open together:
        motion_proxy.post.openHand("LHand")
        motion_proxy.post.openHand("RHand")

        time.sleep(2)  # waiting the movement to finish

        # Closing hands:
        motion_proxy.post.closeHand("LHand")
        motion_proxy.post.closeHand("RHand")

        time.sleep(1)  # waiting the movement to finish

    # The rest() method sends the robot to a relaxed and safe position and sets
    # its motors off. For NAO H25, if the robot is standing, it goes to the
    # Crouch posture and it sets the Stiffness off. It is a blocking call.
    motion_proxy.rest()


if __name__ == "__main__":
    main()