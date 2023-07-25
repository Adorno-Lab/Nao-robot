#!/usr/bin/env python2.7

# More information about controlling the robot walk can be found here:
# http://doc.aldebaran.com/2-1/naoqi/motion/control-walk.html#control-walk

from naoqi import ALProxy

import sys
import time
import math


def main():
    motion_proxy = ALProxy("ALMotion", sys.argv[1], 9559)
    awareness_proxy = ALProxy("ALBasicAwareness", sys.argv[1], 9559)

    awareness_proxy.stopAwareness()  # stopping basic awareness
    motion_proxy.wakeUp()  # waking up the robot
    motion_proxy.moveInit()  # putting the robot in a good initial position

    # Set the destination pose (meters and radians) with respect to FRAME_WORLD:
    x_final = 1.0
    y_final = 0.0
    theta_final = 0

    # Set the tolerance for the desired position:
    error_position = 0.05
    error_orientation = 1 * math.pi / 180

    # First, we will make the robot walk towards the destination position,
    # reducing its velocity as it gets close to the final point. We use the
    # moveToward() method, so the velocity is controlled by the length of the
    # robot's step (the bigger the step length, faster the robot walks).
    try:
        # The moveToward() is a non-blocking method. The most recent command
        # overwrites the previous one, so we use a WHILE loop until the stopping
        # condition is reached.
        while True:
            # Getting current robot pose with respect to FRAME_WORLD:
            current = motion_proxy.getRobotPosition(False)
            x_current = current[0]
            y_current = current[1]
            print "x: ", x_current, " y: ", y_current

            # The moveToward() method accepts the normalized velocity to be
            # applied in each FRAME_ROBOT axis, so the limits are 1 and -1 for
            # positive and negative directions, respectively. The velocity will
            # change according to the distance to the destination point. Here,
            # we multiply the distance by 2 to make the robot start reducing its
            # velocity when it is 0.5m away from the final point in each
            # direction.
            distance_x = x_final - x_current
            distance_y = y_final - y_current
            x = 2*distance_x
            if x > 1:
                x = 1
            if x < -1:
                x = -1
            y = 2*distance_y
            if y > 1:
                y = 1
            if y < -1:
                y = -1

            # Moving the robot to the desired position keeping its orientation:
            motion_proxy.moveToward(x, y, 0)

            # Stopping the robot when it arrives at the destination point with
            # acceptable error:
            if abs(distance_x) < error_position and abs(distance_y) < error_position:
                # Setting the parameters of the moveToward() to zero makes the
                # robot stops more "gracefully" and keeps it stable.
                motion_proxy.moveToward(0, 0, 0)
                break

    except KeyboardInterrupt:
        # Stop the movement and send the robot to the rest posture if there is
        # a keyboard interruption:
        motion_proxy.moveToward(0, 0, 0)
        motion_proxy.waitUntilMoveIsFinished()
        motion_proxy.rest()
        sys.exit(0)

    # Wait until the movement is finished:
    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(1)

    # Turning the robot to the desired orientation:
    try:
        while True:
            current = motion_proxy.getRobotPosition(False)
            theta_current = current[2]
            print "theta (deg): ", current[2] * 180 / math.pi  # in degrees

            # The rotation velocity will start reducing when the angle
            # displacement is 0.5rad (aprox. 28.65 deg).
            theta = 2*(theta_final - theta_current)
            if theta > 1:
                theta = 1
            if theta < -1:
                theta = -1

            motion_proxy.moveToward(0, 0, theta)

            if abs(theta_final - theta_current) < error_orientation:
                motion_proxy.moveToward(0, 0, 0)
                break

    except KeyboardInterrupt:
        motion_proxy.moveToward(0, 0, 0)
        motion_proxy.waitUntilMoveIsFinished()
        motion_proxy.rest()
        sys.exit(0)

    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(2)

    # Now we make the robot go back to the initial position and orientation,
    # matching FRAME_ROBOT with FRAME_WORLD.

    # Turning the robot so it can be 180 degrees with respect to FRAME_WORLD:
    try:
        while True:
            current = motion_proxy.getRobotPosition(False)
            theta_current = current[2]
            print "theta: ", current[2] * 180 / math.pi  # in degrees

            theta = 2*(math.pi - theta_current)
            if theta > 1:
                theta = 1
            if theta < -1:
                theta = -1

            motion_proxy.moveToward(0, 0, theta)

            if abs(math.pi - theta_current) < error_orientation:
                motion_proxy.moveToward(0, 0, 0)
                break

    except KeyboardInterrupt:
        motion_proxy.moveToward(0, 0, 0)
        motion_proxy.waitUntilMoveIsFinished()
        motion_proxy.rest()
        sys.exit(0)

    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(1)

    # Walking towards the initial position:
    try:
        while True:
            current = motion_proxy.getRobotPosition(False)
            x_current = current[0]
            y_current = current[1]
            print "x: ", x_current, " y: ", y_current

            # The robot is with orientation of 180 degrees with respect to the
            # FRAME_WORLD so the x and y axes of the FRAME_ROBOT are in the
            # opposite direction of FRAME_WORLD.
            x = 2*x_current
            if x > 1:
                x = 1
            if x < -1:
                x = -1
            y = 2*y_current
            if y > 1:
                y = 1
            if y < -1:
                y = -1

            motion_proxy.moveToward(x, y, 0)

            if abs(x_current) < error_position and \
                    abs(y_current) < error_position:
                motion_proxy.moveToward(0, 0, 0)
                break

    except KeyboardInterrupt:
        motion_proxy.moveToward(0, 0, 0)
        motion_proxy.waitUntilMoveIsFinished()
        motion_proxy.rest()
        sys.exit(0)

    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(1)

    # Turning the robot to the original orientation:
    try:
        while True:
            current = motion_proxy.getRobotPosition(False)
            theta_current = current[2]
            print "theta: ", current[2] * 180 / math.pi  # in degrees

            theta = 2*(-theta_current)
            if theta > 1:
                theta = 1
            if theta < -1:
                theta = -1

            motion_proxy.moveToward(0, 0, theta)

            if abs(theta_current) < error_orientation:
                motion_proxy.moveToward(0, 0, 0)
                break

    except KeyboardInterrupt:
        motion_proxy.moveToward(0, 0, 0)
        motion_proxy.waitUntilMoveIsFinished()
        motion_proxy.rest()
        sys.exit(0)

    motion_proxy.waitUntilMoveIsFinished()
    time.sleep(1)

    motion_proxy.rest()


if __name__ == "__main__":
    main()