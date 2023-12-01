#!/usr/bin/env python2.7

import argparse
import math
import sys

from naoqi import ALProxy


def main(args):
    motion_proxy = ALProxy("ALMotion", args.robot_ip, args.port)
    awareness_proxy = ALProxy("ALBasicAwareness", args.robot_ip, args.port)

    motion_proxy.wakeUp()  # wake up the robot
    awareness_proxy.stopAwareness()  # stop basic awareness

    # Before sending any walk commands to the robot, use the moveInit()
    # method to put it in a safe initial position, with both its feet
    # flat on the ground.
    motion_proxy.moveInit()

    # There are three high level methods to control the robot walk.
    # By default, the script uses moveTo(), a blocking method that
    # receives the distances along x and y axes and the angle around z
    # axis, relative to FRAME_ROBOT. If there is a commandline choice of
    # method to be used, it overwrites the default.
    #   method 1: moveTo() - blocking, distances and angle
    #   method 2: move() - non-blocking, velocities
    #   method 3: moveToward() - non-blocking, normalized velocities

    #  ======== USING moveTo() METHOD ==================================
    # Method call:
    #   motion_proxy.moveTo(<distance along x>,
    #                       <distance along y>,
    #                       <angle around z>)
    if args.method == 1:
        # The moveTo() method accepts the distance the robot should move
        # along the x and y axes (in meters) and the angle around the z
        # axis (in radians), relative to FRAME_ROBOT.
        try:
            # Since it is a blocking call, we do not need to wait
            # between commands, they are going to be executed only after
            # the previous one is finished.
            motion_proxy.moveTo(0.5, -0.5, -math.pi / 2)
            motion_proxy.moveTo(0, 0, -math.pi / 2)
            motion_proxy.moveTo(0.5, -0.5, -math.pi / 2)
            motion_proxy.moveTo(0, 0, -math.pi / 2)
        except KeyboardInterrupt:
            motion_proxy.moveToward(0, 0, 0)
            motion_proxy.waitUntilMoveIsFinished()
            motion_proxy.rest()
            sys.exit(0)
        except Exception as e:
            print(e)

    #  ======== USING move() METHOD ====================================
    # Method call:
    #   motion_proxy.move(<linear velocity along x>,
    #                     <linear velocity along y>,
    #                     <angular velocity around z>)
    if args.method == 2:
        # The move() method accepts as arguments the linear velocities
        # along x and y axes (in meters per second) and angular velocity
        # around z axis (in radians per second), relative to
        # FRAME_ROBOT. It is a non-blocking call, so the most recent
        # command overwrites the previous one.

        # Setting the target poses (it is the same sequence used with
        # the moveTo method):
        x_target = [0.5, 0, 0.5, 0]
        y_target = [-0.5, 0, -0.5, 0]
        theta_target = [-math.pi / 2, -math.pi / 2,
                        -math.pi / 2, -math.pi / 2]

        # We will make the robot reduce its velocity as it gets close to
        # the target pose, using a while loop and a stopping condition.
        # The stopping condition is going to be the target pose, so we
        # will define tolerance for the robot desired position and
        # orientation.

        # Setting tolerances to the target pose:
        error_position_x = 0.2
        error_position_y = 0.2
        error_orientation = 30 * math.pi / 180

        # Setting the initial velocities (in m/s and rad/s):
        time_periods = [15, 15, 15, 15]  # the time period of each stage
        x_init_vel = [m / s for m, s in zip(x_target, time_periods)]
        y_init_vel = [m / s for m, s in zip(y_target, time_periods)]
        theta_init_vel = [rad / s for rad, s in
                          zip(theta_target, time_periods)]

        for i in range(0, len(x_target)):
            # Getting initial robot pose relative to FRAME_WORLD:
            initial_pose = motion_proxy.getRobotPosition(False)

            while True:
                # Getting current robot pose relative to FRAME_WORLD:
                current_pose = motion_proxy.getRobotPosition(False)

                # Getting displacements:
                delta_x = current_pose[0] - initial_pose[0]
                delta_y = current_pose[1] - initial_pose[1]
                delta_theta = current_pose[2] - initial_pose[2]

                dist = math.sqrt(math.pow(delta_x,2) +
                                 math.pow(delta_y,2))
                target_dist = math.sqrt(math.pow(x_target[i],2) +
                                        math.pow(y_target[i],2))
                # If the position/orientation is inside the tolerance,
                # set the respective velocity to zero. If not, the
                # velocity will be a fraction of the initial value,
                # calculated using the distances.
                if abs(target_dist - dist) < error_position_x:
                    x_vel = 0
                    y_vel = 0
                else:
                    x_vel = x_init_vel[i] * (x_target[i] - delta_x) / \
                            x_target[i]
                    y_vel = y_init_vel[i] * (y_target[i] - delta_y) / \
                            y_target[i]
                if abs(theta_target[i] - delta_theta) < error_orientation:
                    theta_vel = 0
                else:
                    theta_vel = (theta_init_vel[i] *
                                 (theta_target[i] - delta_theta) /
                                 theta_target[i])

                try:
                    motion_proxy.move(x_vel, y_vel, theta_vel)
                except KeyboardInterrupt:
                    motion_proxy.stopMove()
                    motion_proxy.waitUntilMoveIsFinished()
                    motion_proxy.rest()
                    sys.exit(0)
                except Exception as e:
                    print(e)

                # Stopping condition:
                if x_vel == 0 and y_vel == 0 and theta_vel == 0:
                    # Blocking the code until the movement is finished:
                    motion_proxy.waitUntilMoveIsFinished()
                    break

    #  ======== USING moveToward() METHOD ==============================
    # Method call:
    #   motion_proxy.moveToward(<normalized velocity along x>,
    #                           <normalized velocity along y>,
    #                           <normalized velocity around z>)
    if args.method == 3:
        # The moveToward() method accepts as arguments the normalized
        # velocities along x and y axes and around z axis, relative to
        # FRAME_ROBOT. It is a non-blocking call, so the most recent
        # command overwrites the previous one.

        # Setting the target poses:
        x_target = [0.5, 0, 0.5, 0]
        y_target = [-0.5, 0, -0.5, 0]
        theta_target = [-math.pi / 2, -math.pi / 2,
                        -math.pi / 2, -math.pi / 2]

        # As we did with the move() command, we will again make the
        # robot reduce its velocity as it gets close to the target pose,
        # using a while loop and a stopping condition.

        # Setting tolerances to the target pose:
        error_position_x = 0.05
        error_position_y = 0.05
        error_orientation = 5 * math.pi / 180

        for i in range(0, len(x_target)):
            # Getting initial robot pose relative to FRAME_WORLD:
            initial_pose = motion_proxy.getRobotPosition(False)

            while True:
                # Getting current robot pose relative to FRAME_WORLD:
                current_pose = motion_proxy.getRobotPosition(False)

                # Getting displacements:
                delta_x = current_pose[0] - initial_pose[0]
                delta_y = current_pose[1] - initial_pose[1]
                delta_theta = current_pose[2] - initial_pose[2]

                # Again, until the robot is under the tolerance in all
                # axes, the normalized velocity will be reduced using
                # the distances.
                if abs(delta_x) < error_position_x:
                    x_vel = 0
                else:
                    x_vel = (x_target[i] - delta_x) / x_target[i]
                if abs(delta_y) < error_position_y:
                    y_vel = 0
                else:
                    y_vel = (y_target[i] - delta_y) / y_target[i]
                if abs(delta_theta) < error_orientation:
                    theta_vel = 0
                else:
                    theta_vel = ((theta_target[i] - delta_theta) /
                                 theta_target[i])

                try:
                    motion_proxy.moveToward(x_vel, y_vel, theta_vel)
                except KeyboardInterrupt:
                    motion_proxy.moveToward(0, 0, 0)
                    motion_proxy.waitUntilMoveIsFinished()
                    motion_proxy.rest()
                    sys.exit(0)
                except Exception as e:
                    print(e)

                # Stopping condition:
                if x_vel == 0 and y_vel == 0 and theta_vel == 0:
                    # Blocking the code until the movement is finished:
                    motion_proxy.waitUntilMoveIsFinished()
                    break

    motion_proxy.rest()


if __name__ == "__main__":
    # Command line arguments:
    parser = argparse.ArgumentParser(
        description='Example of control of the robot\'s walk. '
                    'Check the tutorial on https://github.com/'
                    'Adorno-Lab/Nao-robot/wiki/Making-NAO-walk')
    parser.add_argument('robot_ip', type=str,
                        help='the IP address of the robot')
    parser.add_argument(
        '-port', type=str, default=9559,
        help='the port on which NAOqi listens (9559 by default)')
    parser.add_argument('-method', type=int, default=1,
                        help='to choose the walk control method to be used: '
                             '"1" (default) to use moveTo() ,'
                             '"2" to use move(), '
                             'and "3" to use moveToward(), ')
    args = parser.parse_args()

    main(args)