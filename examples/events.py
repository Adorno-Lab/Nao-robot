#!/usr/bin/env python2.7

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

import sys
import time

# Global variables to store the modules instances:
HumanGreeter = None
memory = None


# Typically, a module is a class within a library, such as the following one:
class HumanGreeterModule(ALModule):
    """ A module for the robot to say something when people arrive and leave """

    def __init__(self, name):
        # ALModule can be used as a base class for user modules to help serve
        # and advertise their methods.
        ALModule.__init__(self, name)

        # Create a local proxy for each necessary module:
        self.tts = ALProxy("ALTextToSpeech")

        # Subscribe to the FaceDetected event:
        global memory
        # ALMemory provides information about the current state of the robot's
        # actuators and sensors.
        memory = ALProxy("ALMemory")
        memory.subscribeToEvent("PeoplePerception/JustArrived",  # event
                                "HumanGreeter",  # callback module
                                "when_arrive")  # module's callback method
        memory.subscribeToEvent("PeoplePerception/JustLeft",  # event
                                "HumanGreeter",  # callback module
                                "when_leave")  # module's callback method

    # Binding a method, gives you the ability to call 'wait', 'stop',
    # 'isRunning' on it and execute it locally or remotely (from a computer
    # or another robot).
    # This method inside the class will be automatically be bounded if you
    # write a doc string (between the """) and the method name does not
    # start with an underscore (_).

    # To avoid repetitions, unsubscribe to the event while performing
    # action and subscribe again when finished.

    def when_arrive(self, *_args):
        """ Callback method to be called every time a person arrives. """

        memory.unsubscribeToEvent("PeoplePerception/JustArrived",  # event
                                  "HumanGreeter")  # callback module
        self.tts.say("Hello, you")
        memory.subscribeToEvent("PeoplePerception/JustArrived",  # event
                                "HumanGreeter",  # callback module
                                "when_arrive")  # module's callback method

    def when_leave(self, *_args):
        """ Callback method to be called every time a person leaves. """

        memory.unsubscribeToEvent("PeoplePerception/JustLeft",  # event
                                  "HumanGreeter")  # callback module
        self.tts.say("See you")
        memory.subscribeToEvent("PeoplePerception/JustLeft",  # event
                                "HumanGreeter",  # callback module
                                "when_leave")  # module's callback method


def main():
    # A broker is an object that allows you to find modules and methods and to
    # call attached methods to be called outside the process.
    # We need this broker to be able to construct NAOqi modules and subscribe
    # to other modules. The broker must stay alive while the program exists.
    # Keeping the broker alive allows you to create proxies without specifying
    # an IP or port.
    my_broker = ALBroker("myBroker",
                         "0.0.0.0",  # listen to anyone
                         0,  # find a free port and use it
                         sys.argv[1],  # parent broker IP
                         9559)  # parent broker port

    # HumanGreeter *MUST* be a global variable and the same name *MUST* be
    # passed as parameter for the class constructor:
    global HumanGreeter
    HumanGreeter = HumanGreeterModule("HumanGreeter")

    try:
        while True:  # keeping broker alive
            time.sleep(1)
    except KeyboardInterrupt:
        print
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()