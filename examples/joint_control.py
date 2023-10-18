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

    # There are four methods available for joint control of the robot.
    # By default, the script uses angleInterpolation(), a blocking
    # method that receives absolute or relative angles and relative
    # times as parameters. If there is a commandline choice of method to
    # be used, it overwrites the default.
    #   method 1: angleInterpolation() - blocking, absolute or relative
    #                                    angles, relative times
    #   method 2: angleInterpolationWithSpeed() - blocking, absolute
    #                                             angles, speed fraction
    #   method 3: setAngles() - non-blocking, absolute angles, speed
    #                           fraction
    #   method 4: changeAngles() - non-blocking, relative angles,
    #                              speed fraction

    #  ======== USING angleInterpolation() METHOD ======================
    # Method call:
    #   motion_proxy.angleInterpolation(
    #       <joint>,
    #       <target angles>,
    #       <relative times in seconds corresponding to the path points>,
    #       <if the angles are absolute or not>
    #       )
    if args.method == 1:
        # ================= Making the arms move =======================
        arms = ["LArm", "RArm"]
        for arm in arms:
            # The getAngles() method returns the joints or chain angles.
            # The second parameter sets if the sensors will be used to
            # determine the angles.
            initial_angles = motion_proxy.getAngles(arm, False)

            # Setting the target angles of each joint in the arm chain.
            # The target vector should follow the joints order:
            #   [ShoulderPitch,
            #    ShoulderRoll,
            #    ElbowYaw,
            #    ElbowRoll,
            #    WristYaw,
            #    Hand]
            if arm == "LArm":
                target = [[1.6154522895812988, initial_angles[0]],
                          [0.3719860911369324, initial_angles[1]],
                          [-0.7981292605400085, initial_angles[2]],
                          [-1.5378968715667725, initial_angles[3]],
                          [-0.4528430104255676, initial_angles[4]],
                          [0.25, initial_angles[5]]]
            else:
                target = [[1.6155904531478882, initial_angles[0]],
                          [-0.37190526723861694, initial_angles[1]],
                          [0.7983359098434448, initial_angles[2]],
                          [1.537906289100647, initial_angles[3]],
                          [0.45257824659347534, initial_angles[4]],
                          [0.25, initial_angles[5]]]

            times = [[3, 6]]*len(initial_angles)

            # The angleInterpolation() method is a blocking call.
            try:
                motion_proxy.angleInterpolation(arm, target, times, True)
            except KeyboardInterrupt:
                pass
            except Exception as e:
                print(e)

        # ================= Making the head move =======================
        initial_angle = motion_proxy.getAngles("HeadYaw", False)
        target = [30 * math.pi / 180, initial_angle[0],
                  -30 * math.pi / 180, initial_angle[0]]

        try:
            motion_proxy.angleInterpolation("HeadYaw", target,  [3, 6, 9, 12],
                                            True)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

    #  ======== USING angleInterpolationWithSpeed() METHOD =============
    # Method call:
    #   motion_proxy.angleInterpolationWithSpeed(
    #       <joint>,
    #       <target angles>,
    #       <fraction of maximum speed to use>
    #       )
    if args.method == 2:
        # ================= Making the arms move =======================
        arms = ["LArm", "RArm"]
        for arm in arms:
            initial_angles = motion_proxy.getAngles(arm, False)

            if arm == "LArm":
                target = [1.6154522895812988,
                          0.3719860911369324,
                          -0.7981292605400085,
                          -1.5378968715667725,
                          -0.4528430104255676,
                          0.25]
            else:
                target = [1.6155904531478882,
                          -0.37190526723861694,
                          0.7983359098434448,
                          1.537906289100647,
                          0.45257824659347534,
                          0.25]

            # The angleInterpolationWithSpeed() method does not accept
            # multiple target angles in one single method call, so we
            # use a FOR loop, but since it is a blocking call we do not
            # need time.sleep().
            targets = [target, initial_angles]
            try:
                for target in targets:
                    motion_proxy.angleInterpolationWithSpeed(arm, target, 0.1)
            except KeyboardInterrupt:
                pass
            except Exception as e:
                print(e)

        # ================= Making the head move =======================
        initial_angle = motion_proxy.getAngles("HeadYaw", False)
        targets = [30 * math.pi / 180, initial_angle[0],
                   -30 * math.pi / 180, initial_angle[0]]

        try:
            for target in targets:
                motion_proxy.angleInterpolationWithSpeed("HeadYaw", target,
                                                         0.1)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

    #  ======== USING setAngles() METHOD ===============================
    # Method call:
    #   motion_proxy.setAngles(
    #       <joint>,
    #       <target angles>,
    #       <fraction of maximum speed to use>
    #       )
    if args.method == 3:
        # ================= Making the arms move =======================
        arms = ["LArm", "RArm"]
        for arm in arms:
            initial_angles = motion_proxy.getAngles(arm, False)

            if arm == "LArm":
                target = [1.6154522895812988,
                          0.3719860911369324,
                          -0.7981292605400085,
                          -1.5378968715667725,
                          -0.4528430104255676,
                          0.25]
            else:
                target = [1.6155904531478882,
                          -0.37190526723861694,
                          0.7983359098434448,
                          1.537906289100647,
                          0.45257824659347534,
                          0.25]

            # The setAngles() method is a non-blocking call.
            targets = [target, initial_angles]
            try:
                for target in targets:
                    motion_proxy.setAngles(arm, target, 0.1)
                    time.sleep(2)
            except KeyboardInterrupt:
                pass
            except Exception as e:
                print(e)

        # ================= Making the head move =======================
        initial_angle = motion_proxy.getAngles("HeadYaw", False)
        targets = [30 * math.pi / 180, initial_angle[0],
                   -30 * math.pi / 180, initial_angle[0]]

        try:
            for target in targets:
                motion_proxy.setAngles("HeadYaw", target, 0.1)
                time.sleep(2)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

    #  ======== USING changeAngles() METHOD ============================
    # Method call:
    #   motion_proxy.changeAngles(
    #       <joint>,
    #       <target angles>,
    #       <fraction of maximum speed to use>
    #       )
    if args.method == 4:
        # ================= Making the head move =======================
        targets = [30 * math.pi / 180, -30 * math.pi / 180,
                   -30 * math.pi / 180, 30 * math.pi / 180]

        # The changeAngles() method is a non-blocking call.
        try:
            for target in targets:
                motion_proxy.changeAngles("HeadYaw", target, 0.25)
                time.sleep(3)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

    # ======== MOVING HANDS ============================================
    if args.hand == 1:
        # Opening hands using the argument POST, so they can open
        # together:
        try:
            motion_proxy.post.openHand("LHand")
            motion_proxy.post.openHand("RHand")
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

        time.sleep(2)  # waiting for the movement to finish

        # Closing hands:
        try:
            motion_proxy.post.closeHand("LHand")
            motion_proxy.post.closeHand("RHand")
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

        time.sleep(1)  # waiting for the movement to finish

    # The rest() method sends the robot to a relaxed and safe position
    # and sets its motors off. For NAO H25, if the robot is standing, it
    # goes to the Crouch posture and it sets the Stiffness off. It is a
    # blocking call.
    motion_proxy.rest()


if __name__ == "__main__":
    # Command line arguments:
    parser = argparse.ArgumentParser(
        description='Example of joint control of the robot\'s arms, head, and '
                    'hands. Check the tutorial on https://github.com/'
                    'Adorno-Lab/Nao-robot/wiki/Making-NAO-move')
    parser.add_argument('robot_ip', type=str,
                        help='the IP address of the robot')
    parser.add_argument(
        '-port', type=str, default=9559,
        help='the port on which NAOqi listens (9559 by default)')
    parser.add_argument('-method', type=int, default=1,
                        help='to choose the joint control method to be used: '
                             '"1" (default) to use angleInterpolation() ,'
                             '"2" to use angleInterpolationWithSpeed(), '
                             '"3" to use setAngles(), '
                             'and "4" to use changeAngles()')
    parser.add_argument('-hand', type=int, default=0,
                        help='to indicate if the robot should open and close '
                             'its hands (1) or not (0) (default).')
    args = parser.parse_args()

    main(args)