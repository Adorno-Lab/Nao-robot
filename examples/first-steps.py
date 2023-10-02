#!/usr/bin/env python2.7

# Check the tutorial "First steps programming NAO (Python and Ubuntu)" on
# https://github.com/Adorno-Lab/Nao-robot/wiki/First-steps-programming-NAO-(Python-and-Ubuntu)

# To execute this script, run on a terminal:
#   $ python2.7 first-steps.py "<robot-ip-address>"

from naoqi import ALProxy
import math
import argparse

parser = argparse.ArgumentParser(description='First steps programming NAO. Check the tutorial on '
                                             'https://github.com/Adorno-Lab/Nao-robot/wiki/First-steps-programming-NAO-'
                                             '(Python-and-Ubuntu)')
parser.add_argument('robot_ip', type=str, help='the IP address of the robot')
args = parser.parse_args()

try:
    # Getting robot's IP address from the command line:
    robot_ip = args.robot_ip

    # ALProxy(name, ip, port)
    #   name: the name of the module
    #   ip: the ip of the robot
    #   port: the port on which NAOqi listens (9559 by default)

    # ------- Making NAO speak:
    tts = ALProxy("ALTextToSpeech", robot_ip, 9559)
    tts.say("Hello, world!")

    # ------- Making NAO move:
    motion = ALProxy("ALMotion", robot_ip, 9559)
    # The robot will not move unless you set the stiffness of the joints to
    # something that is not 0.
    motion.setStiffnesses("Body", 1.0)
    motion.moveInit()  # to put the robot in a correct position first

    # The method moveTo(x, y, theta) moves the robot to a given pose in the ground
    # plane, relative to the robot frame, where
    #   x: distance along the X axis (front) in meters
    #   y: distance along the Y axis (left) in meters
    #   theta: rotation around the Z axis (up) in radians.
    # Every proxy you create has an attribute named POST.
    # We will call the method moveTo using the POST attribute so we can wait until
    # it finishes before continuing. But it could have been only motion.moveTo().
    id_motion = motion.post.moveTo(0.3, 0, math.pi/3)

    # We use the id returned by the post usage as parameter for the WAIT method.
    motion.wait(id_motion, 0)

    # ------- Making NAO move and speak at the same time:
    # We also use the POST attribute to call long methods in the background.
    motion.post.moveTo(0.1, 0, 0)
    tts.say("I'm walking and saying a long sentence at the same time")

except KeyboardInterrupt:
    pass
except Exception as e:
    print(e)