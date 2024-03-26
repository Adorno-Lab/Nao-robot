from naoqi import ALProxy


def main():
    ip = "192.168.0.100"
    motion_proxy = ALProxy("ALMotion", ip, 9559)
    posture_proxy = ALProxy("ALRobotPosture", ip, 9559)
    awareness_proxy = ALProxy("ALBasicAwareness", ip, 9559)

    motion_proxy.wakeUp()
    awareness_proxy.stopAwareness()
    posture_proxy.goToPosture("StandZero", 0.5)

    # If useSensorValues is False, it does not use the sensors to get the pose,
    # which means it considers the desired joint values and calculates the pose
    # We do not use the sensors to check the model.
    pose = motion_proxy.getPosition("CameraTop", 0, False)
    print(pose)

    motion_proxy.rest()


if __name__ == "__main__":
    main()