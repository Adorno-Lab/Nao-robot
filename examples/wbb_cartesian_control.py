#!/usr/bin/env python2.7

import argparse
import math
import time

from naoqi import ALProxy
import motion


def main(args):
    motion_proxy = ALProxy("ALMotion", args.robot_ip, args.port)
    posture_proxy = ALProxy("ALRobotPosture", args.robot_ip, args.port)

    # Waking up the robot sets its motors on. For NAO H25, it sets the
    # Stiffness on and keeps the current position. If it starts from the
    # Crouch posture, it makes the robot stand up; if it starts already
    # standing or sitting, it does nothing with the robot posture. It is
    # a blocking call.
    motion_proxy.wakeUp()

    # The StandInit posture is a good starting point for most of the
    # motion animations. We use the ALRobotPosture module to send the
    # robot to predefined postures such as StandInit. It is a blocking
    # call.
    posture_proxy.goToPosture("StandInit", 0.5)

    # ======== MAKING THE ARMS MOVE ====================================
    arms = ["LArm", "RArm"]
    for arm in arms:
        # Activate arm tracking (it implicitly activates Whole Body
        # Balancer):
        motion_proxy.wbEnableEffectorControl(arm, True)

        # The getPosition method returns a vector containing the pose
        # (x, y, z, wx, wy, wz) of any joint, chain, or sensor in meters
        # and radians, relative to the chosen frame. The last parameter
        # sets if the sensors values will be used to determine the pose.
        initial_pose = motion_proxy.getPosition(arm, motion.FRAME_ROBOT, False)

        # An "Effector" is a predefined 3D point in the robot and it's
        # generally the end of a chain. The points related to "LArm" and
        # "RArm" are inside the respective hand.

        # To set the target position of the arm, the parameters are the
        # x, y, and z axis coordinates in meters relative to the
        # FRAME_ROBOT. The method wbSetEffectorControl is non blocking,
        # so we use time.sleep to allow the arm to arrive at the target
        # position. The recommended minimum period between two
        # successive commands is 0.2 seconds.
        # - Feasible arm positions (according to an Aldebaran script):
        #       x-axis: [0.0, 0.12] meters
        #       y-axis LArm: [-0.05, 0.10] meters
        #       y-axis RArm: [-0.10, 0.05] meters
        #       z-axis: [-0.10, 0.10] meters
        if arm == "LArm":
            target = [0.06, 0.05, 0.3]
        else:
            target = [0.06, -0.05, 0.3]

        try:
            motion_proxy.wbSetEffectorControl(arm, target)
            time.sleep(3)
            motion_proxy.wbSetEffectorControl(arm, initial_pose[0:3])
            time.sleep(3)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

        # Deactivate arm tracking:
        motion_proxy.wbEnableEffectorControl(arm, False)

    # ======== MAKING THE HEAD MOVE ====================================
    motion_proxy.wbEnableEffectorControl("Head", True)

    # - Feasible head orientations (according to an Aldebaran script):
    #       x axis: [-20.0, +20.0] degrees
    #       y axis: [-75.0, +70.0] degrees
    #       z axis: [-30.0, +30.0] degrees
    targets = [[0, 0, 30*math.pi/180], [0, 0, 0],
               [0, 0, -30*math.pi/180], [0, 0, 0]]

    for target in targets:
        try:
            motion_proxy.wbSetEffectorControl("Head", target)
            time.sleep(3)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)

    motion_proxy.wbEnableEffectorControl("Head", False)

    # The rest() method sends the robot to a relaxed and safe position
    # and sets its motors off. For NAO H25, if the robot is standing, it
    # goes to the Crouch posture and it sets the Stiffness off. It is a
    # blocking call.
    motion_proxy.rest()


if __name__ == "__main__":
    # Command line arguments:
    parser = argparse.ArgumentParser(
        description='Example of cartesian control of the robot\'s arms and '
                    'head using the Whole Body Balancer, a tool to generate '
                    'safe and natural motion. Check the tutorial on https://'
                    'github.com/Adorno-Lab/Nao-robot/wiki/Making-NAO-move')
    parser.add_argument('robot_ip', type=str,
                        help='the IP address of the robot')
    parser.add_argument(
        '-port', type=str, default=9559,
        help='the port on which NAOqi listens (9559 by default)')
    args = parser.parse_args()

    main(args)