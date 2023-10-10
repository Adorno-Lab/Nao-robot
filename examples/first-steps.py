#!/usr/bin/env python2.7
import argparse
import math

from naoqi import ALProxy

# Command line arguments:
parser = argparse.ArgumentParser(
    description='First steps programming NAO. Check the tutorial on '
                'https://github.com/Adorno-Lab/Nao-robot/wiki/First-steps-'
                'programming-NAO-(Python-and-Ubuntu)')
parser.add_argument('robot_ip', type=str, help='the IP address of the robot')
parser.add_argument('-port', type=str, default=9559,
                    help='the port on which NAOqi listens (9559 by default)')
args = parser.parse_args()

# Usage: ALProxy(name, ip, port)
#   name: the name of the module
#   ip: the ip of the robot
#   port: the port on which NAOqi listens (9559 by default)

# ======== MAKING NAO SPEAK ============================================
try:
    TTSProxy = ALProxy("ALTextToSpeech", args.robot_ip, args.port)
    TTSProxy.say("Hello, world!")
except KeyboardInterrupt:
    pass
except Exception as e:
    print("Error while making NAO speak")
    print(e)

# ======== MAKING NAO MOVE =============================================
try:
    motionProxy = ALProxy("ALMotion", args.robot_ip, args.port)
    # The robot will not move unless you set the stiffness of the joints
    # to something that is not 0:
    motionProxy.setStiffnesses("Body", 1.0)
    # Put the robot in a correct position first:
    motionProxy.moveInit()

    # The method moveTo(x, y, theta) moves the robot to a given pose in
    # the ground plane, relative to the robot frame, where
    #   x: distance along the X axis (front) in meters
    #   y: distance along the Y axis (left) in meters
    #   theta: rotation around the Z axis (up) in radians.
    # Every proxy you create has an optional attribute named POST. We
    # will use it, so we can wait until it finishes before continuing.
    id_motion = motionProxy.post.moveTo(0.3, 0, math.pi/3)

    # We use the id from the POST usage as the parameter for the WAIT
    # method.
    motionProxy.wait(id_motion, 0)
except KeyboardInterrupt:
    pass
except Exception as e:
    print("Error while making NAO move")
    print(e)

# ========  MAKING NAO MOVE AND SPEAK AT THE SAME TIME =================
try:
    # We also use the POST attribute to call long methods in the
    # background.
    motionProxy.post.moveTo(0.1, 0, 0)
    TTSProxy.say("I'm walking and saying a long sentence at the same time")
except KeyboardInterrupt:
    pass
except Exception as e:
    print("Error while making NAO move and speak at the same time")
    print(e)