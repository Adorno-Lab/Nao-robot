#!/usr/bin/env python2.7

# This is a script with examples of
#   how to create a module,
#   how to react to events, and
#   how to make the robot recognize words said by the human.
# It includes
#   creating a Broker and keeping it alive,
#   automatically binding methods,
#   subscribing to events, and
#   getting data from the ALMemory module.

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

import sys
import time

# Global variables to store the modules instances:
React = None  # module that will be created
memory = None  # ALMemory module
speech_recognition = None  # ALSpeechRecognition module


# Typically, a module is a class within a library, such as the following one:
class ReactModule(ALModule):
    """ A module for the robot to react to some events. """

    def __init__(self, name):
        # ALModule can be used as a base class for user modules to help serve
        # and advertise their methods.
        ALModule.__init__(self, name)

        # Create a local proxy for each necessary module:
        self.tts = ALProxy("ALTextToSpeech")

        # ALMemory provides information about the current state of the robot's
        # actuators and sensors.
        global memory
        memory = ALProxy("ALMemory")

        # Subscribe to the desired events:
        memory.subscribeToEvent("PeoplePerception/JustArrived",  # event
                                "React",  # callback module
                                "when_arrive")  # module's callback method
        memory.subscribeToEvent("PeoplePerception/JustLeft",  # event
                                "React",  # callback module
                                "when_leave")  # module's callback method
        memory.subscribeToEvent("WordRecognized",  # event
                                "React",  # callback module
                                "when_speech_recognized")  # module's callback method

        # Setting up the speech recognition module:
        global speech_recognition
        speech_recognition = ALProxy("ALSpeechRecognition")
        speech_recognition.pause(True)  # pause before setting vocabulary
        speech_recognition.setLanguage("English")
        vocabulary = ["robot"]  # list of words for the vocabulary
        speech_recognition.setVocabulary(vocabulary, False)  # second parameter is to enable or disable word spotting
        speech_recognition.pause(False)
        speech_recognition.subscribe("SRSubscriber")  # start writing in ALMemory's "WordRecognized"

        # Define the minimum probability for a word to be considered recognized:
        self.word_probability = 0.5

    # Binding a method, gives you the ability to call 'wait', 'stop',
    # 'isRunning' on it and execute it locally or remotely (from a computer
    # or another robot).
    # A class method will be automatically bounded if it has a doc string
    # (between the """) and the method name does not start with underscore (_).

    # To avoid repetitions, unsubscribe to the event while performing some
    # action and subscribe again when finished.

    def when_arrive(self, *_args):
        """ Callback method to be called every time a person arrives. """

        memory.unsubscribeToEvent("PeoplePerception/JustArrived",  # event
                                  "React")  # callback module
        self.tts.say("Hello, you")
        memory.subscribeToEvent("PeoplePerception/JustArrived",  # event
                                "React",  # callback module
                                "when_arrive")  # module's callback method

    def when_leave(self, *_args):
        """ Callback method to be called every time a person leaves. """

        memory.unsubscribeToEvent("PeoplePerception/JustLeft",  # event
                                  "React")  # callback module
        self.tts.say("See you")
        memory.subscribeToEvent("PeoplePerception/JustLeft",  # event
                                "React",  # callback module
                                "when_leave")  # module's callback method

    def when_speech_recognized(self, *_args):
        """ Callback method to be called every time speech is recognized. """

        memory.unsubscribeToEvent("WordRecognized",  # event
                                  "React")  # callback module

        # Getting WordRecognized data from memory:
        speech_data = memory.getData("WordRecognized")
        # The WordRecognized key is organized as [ph_1, pb_1, ..., ph_n, pb_n],
        # where ph_i is one phrase from the vocabulary and pb_i is the estimated
        # probability that ph_i has been said by the human. The order inside the
        # vector is from the most likely phrases to the less likely.
        if speech_data[0] == "robot" and speech_data[1] > self.word_probability:
            self.tts.say("human")

        memory.subscribeToEvent("WordRecognized",  # event
                                "React",  # callback module
                                "when_speech_recognized")  # module's callback method


def main():
    # A broker is an object that allows you to find modules and methods and to
    # call attached methods from outside the process.
    # We need this broker to be able to construct NAOqi modules and subscribe
    # to other modules. The broker must stay alive while the program exists.
    # Keeping the broker alive allows you to create proxies without specifying
    # an IP or port.
    my_broker = ALBroker("myBroker",
                         "0.0.0.0",  # listen to anyone
                         0,  # find a free port and use it
                         sys.argv[1],  # parent broker IP
                         9559)  # parent broker port

    # React *MUST* be a global variable and the same name *MUST* be passed as
    # parameter for the class constructor.
    global React
    React = ReactModule("React")

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