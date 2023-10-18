#!/usr/bin/env python2.7

import argparse
import math
import time

from naoqi import ALProxy


def main(args):

    motion_proxy = ALProxy("ALMotion", args.robot_ip, args.port)
    posture_proxy = ALProxy("ALRobotPosture", args.robot_ip, args.port)
    awareness_proxy = ALProxy("ALBasicAwareness", args.robot_ip, args.port)

    # Waking up the robot sets its motors on. For NAO H25, it sets the
    # Stiffness on and keeps the current position. If it starts from the
    # Crouch posture, it makes the robot stand up; if it starts already
    # standing or sitting, it does nothing with the robot posture. It is
    # a blocking call.
    motion_proxy.wakeUp()

    # The ALBasicAwareness module enables the robot to be aware of
    # stimuli coming from its surrounding environment.
    awareness_proxy.stopAwareness()

    # The StandInit posture is a good starting point for most of the
    # motion animations. We use the ALRobotPosture module to send the
    # robot to predefined postures such as StandInit. It is a blocking
    # call.
    posture_proxy.goToPosture("StandInit", 0.5)

    # Defining variables to hold the number relative to each frame:
    FRAME_TORSO = 0
    FRAME_WORLD = 1
    FRAME_ROBOT = 2

    # There are four methods available for cartesian control of the
    # robot. By default, the script uses positionInterpolations(), a
    # blocking method that receives pose vectors and relative times as
    # parameters. If there is a commandline choice of method to be used,
    # it overwrites the default.
    #   method 1: positionInterpolations() - blocking, pose, relative times
    #   method 2: transformInterpolations() - blocking, HTM, relative times
    #   method 3: setPositions() - non-blocking, pose, speed fraction
    #   method 4: setTransforms() - non-blocking, HTM, speed fraction

    #  ======== USING positionInterpolations() METHOD ==================
    # Method call:
    #   motion_proxy.positionInterpolations(
    #       <effector>,
    #       <frame>,
    #       <target vector>,
    #       <integer indicating what to control>,
    #       <relative times in seconds corresponding to the path points>
    #       )
    if args.method == 1:
        # ================= Making the arms move =======================
        arms = ["LArm", "RArm"]
        for arm in arms:
            # The getPosition() method returns a vector containing the
            # pose (x, y, z, wx, wy, wz) of any joint, chain, or sensor
            # in meters and radians, relative to the chosen frame. The
            # last parameter sets if the sensors will be used to
            # determine the pose.
            initial_pose = motion_proxy.getPosition(arm, FRAME_ROBOT, False)

            # To set the target position of the arm, the parameters are
            # position and orientation (x, y, z, wx, wy, wz) in meters
            # and radians.
            if arm == "LArm":
                target = [0.06, 0.05, 0.3,
                          initial_pose[3], initial_pose[4], initial_pose[5]]
            else:
                target = [0.06, -0.05, 0.3,
                          initial_pose[3], initial_pose[4], initial_pose[5]]

            # The robot arms do not have enough degrees of freedom to
            # enable pose (position + orientation) control, so a
            # parameter in the positionInterpolations method sets what
            # will be controlled. It is an integer, obtained with the
            # sum of numbers associated to each one of the translation
            # and rotation axes:
            t_x = 1
            t_y = 2
            t_z = 4
            r_wx = 8
            r_wy = 16
            r_wz = 32
            # For example, if you want to control only position
            # (x, y, z), the parameter should be t_x + t_y + t_z = 7.
            # The positionInterpolations method is a blocking call.
            try:
                motion_proxy.positionInterpolations(arm, FRAME_ROBOT,
                                                    [target, initial_pose],
                                                    t_x + t_y + t_z,  [3, 6])
            except KeyboardInterrupt:
                pass
            except Exception as e:
                print(e)

        # ================= Making the head move =======================
        initial_pose = motion_proxy.getPosition("Head", FRAME_ROBOT, False)

        targets = [
            [initial_pose[0], initial_pose[1], initial_pose[2],
                0, 0, 30 * math.pi / 180],
            [initial_pose[0], initial_pose[1], initial_pose[2], 0, 0, 0],
            [initial_pose[0], initial_pose[1], initial_pose[2],
                0, 0, -30 * math.pi / 180],
            [initial_pose[0], initial_pose[1], initial_pose[2],
                0, 0, 0]]
        try:
            motion_proxy.positionInterpolations("Head", FRAME_ROBOT, targets,
                                                r_wx + r_wy + r_wz,
                                                [3, 6, 9, 12])
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

    #  ======== USING transformInterpolations() METHOD =================
    # Method call:
    #   motion_proxy.transformInterpolations(
    #       <effector>,
    #       <frame>,
    #       <target transform>,
    #       <integer indicating what to control>,
    #       <relative times in seconds corresponding to the path points>
    #       )
    if args.method == 2:
        # ================= Making the arms move =======================
        arms = ["LArm", "RArm"]
        for arm in arms:
            # The getTransform() method returns a homogeneous
            # transformation matrix (HTM) to any joint, chain, or sensor
            # relative to the chosen frame. The last parameter sets if
            # the sensors will be used to determine the transformation.
            initial_transform = motion_proxy.getTransform(arm, FRAME_ROBOT,
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

            # The transformInterpolations() method is a blocking call.
            try:
                motion_proxy.transformInterpolations(
                    arm, FRAME_ROBOT, [target, initial_transform],
                    t_x + t_y + t_z, [3, 6])
            except KeyboardInterrupt:
                pass
            except Exception as e:
                print(e)

        # ================= Making the head move =======================
        initial_transform = motion_proxy.getTransform("Head", FRAME_ROBOT,
                                                      False)

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

        try:
            motion_proxy.transformInterpolations("Head", FRAME_ROBOT, target,
                                                 r_wx + r_wy + r_wz,
                                                 [3, 6, 9, 12])
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

    #  ======== USING setPositions() METHOD ============================
    # Method call:
    #   motion_proxy.setPositions(
    #       <effector>,
    #       <frame>,
    #       <target vector>,
    #       <fraction of maximum speed to use>,
    #       <integer indicating what to control>
    #       )
    if args.method == 3:
        # ================= Making the arms move =======================
        arms = ["LArm", "RArm"]
        for arm in arms:
            initial_pose = motion_proxy.getPosition(arm, FRAME_ROBOT, False)

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

            # The setPositions() method is a non-blocking call.
            try:
                motion_proxy.setPositions(arm, FRAME_ROBOT, target, 0.25,
                                          t_x + t_y + t_z)
                time.sleep(3)
                motion_proxy.setPositions(arm, FRAME_ROBOT, initial_pose, 0.25,
                                          t_x + t_y + t_z)
                time.sleep(3)
            except KeyboardInterrupt:
                pass
            except Exception as e:
                print(e)

        # ================= Making the head move =======================
        initial_pose = motion_proxy.getPosition("Head", FRAME_ROBOT, False)

        targets = [[initial_pose[0], initial_pose[1], initial_pose[2],
                    0, 0, 30 * math.pi / 180],
                   [initial_pose[0], initial_pose[1], initial_pose[2], 0, 0, 0],
                   [initial_pose[0], initial_pose[1], initial_pose[2],
                    0, 0, -30 * math.pi / 180],
                   [initial_pose[0], initial_pose[1], initial_pose[2], 0, 0, 0]]

        try:
            for target in targets:
                motion_proxy.setPositions("Head", FRAME_ROBOT, target, 0.25,
                                          r_wx + r_wy + r_wz)
                time.sleep(3)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

    #  ======== USING setTransforms() METHOD ===========================
    # Method call:
    #   motion_proxy.setTransforms(
    #       <effector>,
    #       <frame>,
    #       <target transform>,
    #       <fraction of maximum speed to use>,
    #       <integer indicating what to control>
    #       )
    if args.method == 4:
        # ================= Making the arms move =======================
        arms = ["LArm", "RArm"]
        for arm in arms:
            initial_transform = motion_proxy.getTransform(arm, FRAME_ROBOT,
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

            # The setTransforms() method is a non-blocking call.
            try:
                motion_proxy.setTransforms(arm, FRAME_ROBOT, target, 0.5,
                                           t_x + t_y + t_z)
                time.sleep(3)
                motion_proxy.setTransforms(arm, FRAME_ROBOT, initial_transform,
                                           0.25, t_x + t_y + t_z)
                time.sleep(3)
            except KeyboardInterrupt:
                pass
            except Exception as e:
                print(e)

        # ================= Making the head move =======================
        initial_transform = motion_proxy.getTransform("Head", FRAME_ROBOT,
                                                      False)

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

        try:
            for target in targets:
                motion_proxy.setTransforms("Head", FRAME_ROBOT, target, 0.25,
                                           r_wx + r_wy + r_wz)
                time.sleep(3)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

    # The rest() method sends the robot to a relaxed and safe position
    # and sets its motors off. For NAO H25, if the robot is standing, it
    # goes to the Crouch posture and it sets the Stiffness off. It is a
    # blocking call.
    motion_proxy.rest()


if __name__ == "__main__":
    # Command line arguments:
    parser = argparse.ArgumentParser(
        description='Example of cartesian control of the robot\'s arms and '
                    'head. Check the tutorial on https://github.com/'
                    'Adorno-Lab/Nao-robot/wiki/Making-NAO-move')
    parser.add_argument('robot_ip', type=str,
                        help='the IP address of the robot')
    parser.add_argument(
        '-port', type=str, default=9559,
        help='the port on which NAOqi listens (9559 by default)')
    parser.add_argument('-method', type=int, default=1,
                        help='to choose the cartesian method to be used: '
                             '"1" (default) to use positionInterpolations() ,'
                             '"2" to use transformInterpolations(), '
                             '"3" to use setPositions(), '
                             'and "4" to use setTransforms()')
    args = parser.parse_args()

    main(args)