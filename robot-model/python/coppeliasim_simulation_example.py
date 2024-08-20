# Example of simulation of the NAO robot using CoppeliaSim. It creates
# an object for each chain. The arms configurations change from the initial
# configuration (pointing forwards) to both arms pointing upwards.

# Before running:
#   - Open CoppeliaSim
#   - Open the 'nao_scene.ttt' or add a new NAO robot to a blank scene

from math import pi
import matplotlib.pyplot as plt
import numpy as np
import time

from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface

import NaoRobot

# Create DQ_VrepInterface object to start communication with CoppeliaSim
vi = DQ_VrepInterface()

# Create robot objects and define joints names in CoppeliaSim
topcamera = NaoRobot.TopCamera()
q_topcamera = [0, 0]
joints_names_topcamera = []
for j in topcamera.get_joints_names():
    joints_names_topcamera.append("/NAO/" + j)

bottomcamera = NaoRobot.BottomCamera()
q_bottomcamera = [0, 0]
joints_names_bottomcamera = []
for j in bottomcamera.get_joints_names():
    joints_names_bottomcamera.append("/NAO/" + j)

leftarm = NaoRobot.LeftArm()
q_leftarm = [0, 0, 0, 0, 0]
joints_names_leftarm = []
for j in leftarm.get_joints_names():
    joints_names_leftarm.append("/NAO/" + j)

rightarm = NaoRobot.RightArm()
q_rightarm = [0, 0, 0, 0, 0]
joints_names_rightarm = []
for j in rightarm.get_joints_names():
    joints_names_rightarm.append("/NAO/" + j)

leftleg = NaoRobot.LeftLeg()
q_leftleg = [0, 0, 0, 0, 0, 0]
joints_names_leftleg = []
for j in leftleg.get_joints_names():
    joints_names_leftleg.append("/NAO/" + j)

rightleg = NaoRobot.RightLeg()
q_rightleg = [0, 0, 0, 0, 0, 0]
joints_names_rightleg = []
for j in rightleg.get_joints_names():
    joints_names_rightleg.append("/NAO/" + j)

# Always use a try-catch, because otherwise in case the connection with
# CoppeliaSim is lost, the clientid will be locked for future use
try:
    # Finish any previous CoppeliaSim communication
    vi.stop_simulation()
    vi.disconnect_all()

    # Start a new connection
    # vi.connect(port,
    #            timeout_in_ms,
    #            number_of_retries_for_each_method_call)
    vi.connect(19997, 100, 10)
    print("Communication established")

    # Start simulation
    vi.start_simulation()
    print("Simulation started")

    # Define target angles for the arms chains (both arms up)
    q_target_leftarm = [-pi/2, 0, 0, 0, 0]
    q_target_rightarm = [-pi/2, 0, 0, 0, 0]

    # 10 steps to reach targets
    i = 0
    while i < 10:
        q_leftarm = [i*j/10 for j in q_target_leftarm]
        q_rightarm = [i*j/10 for j in q_target_rightarm]

        time.sleep(0.1)
        i = i + 1
        vi.set_joint_target_positions(joints_names_leftarm, q_leftarm)
        vi.set_joint_target_positions(joints_names_rightarm, q_rightarm)

    time.sleep(2.0)

    # Finish CoppeliaSim communication
    vi.stop_simulation()
    print("Simulation finished")
    vi.disconnect_all()
    print("Communication finished")

except Exception as exp:
    print(exp)
    vi.disconnect_all()