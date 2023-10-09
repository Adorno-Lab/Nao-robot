#!/usr/bin/env python2.7

import argparse
import time
import sys

from naoqi import ALProxy, ALBroker, ALModule

# Global variables to store the modules instances.
react = None  # instance of the module that will be created


# Typically, a module is a class within a library. We will create one
# to react to some events.
class reactModule(ALModule):
    """A module for the robot to react to some events."""

    def __init__(self, name):
        # ALModule can be used as a base class for user modules to help
        # serve and advertise their methods.
        ALModule.__init__(self, name)

        # Create a proxy for each necessary module.
        self.TTSProxy = ALProxy("ALTextToSpeech")
        self.memoryProxy = ALProxy("ALMemory")
        self.speechRecognitionProxy = ALProxy("ALSpeechRecognition")

        # Subscribe to the desired events using
        # memoryProxy.subscribeToEvent(<event>,
        #                              <callback_module>,
        #                              <callback_method_from_module>)
        self.memoryProxy.subscribeToEvent("PeoplePerception/JustArrived",
                                          "react", "when_arrive")
        self.memoryProxy.subscribeToEvent("PeoplePerception/JustLeft", "react",
                                          "when_leave")
        self.memoryProxy.subscribeToEvent("WordRecognized", "react",
                                          "when_speech_recognized")

        # Set up the speech recognition proxy.
        # Add a pause before setting the vocabulary:
        self.speechRecognitionProxy.pause(True)
        self.speechRecognitionProxy.setLanguage("English")
        # Define the list of words for the vocabulary:
        vocabulary = ["human"]
        # The second parameters of setVocabulary() is to enable or
        # disable word spotting:
        self.speechRecognitionProxy.setVocabulary(vocabulary, False)
        # Enable the recognition again:
        self.speechRecognitionProxy.pause(False)
        # Start writing in ALMemory's "WordRecognized"
        self.speechRecognitionProxy.subscribe("SRSubscriber")

        # Define the minimum probability for a word to be considered
        # recognized:
        self.word_probability = 0.5

    # Binding a method, gives you the ability to call 'wait', 'stop',
    # 'isRunning' on it and execute it locally or remotely (from a
    # computer or another robot).
    # A class method will be automatically bound if it has a docstring
    # (between the """) and the method name does not start with an
    # underscore (_).

    # To avoid repetitions, unsubscribe to the event while performing
    # some action and subscribe again when finished. Unsubscribe using:
    # memoryProxy.unsubscribeToEvent(<event>,
    #                                <callback_module>)

    def when_arrive(self, *_args):
        """Callback method to be called every time a person arrives."""
        self.memoryProxy.unsubscribeToEvent("PeoplePerception/JustArrived",
                                            "react")
        self.TTSProxy.say("Hello, you")
        self.memoryProxy.subscribeToEvent("PeoplePerception/JustArrived",
                                          "react", "when_arrive")

    def when_leave(self, *_args):
        """Callback method to be called every time a person leaves."""
        self.memoryProxy.unsubscribeToEvent("PeoplePerception/JustLeft",
                                            "react")
        self.TTSProxy.say("See you")
        self.memoryProxy.subscribeToEvent("PeoplePerception/JustLeft","react",
                                          "when_leave")

    def when_speech_recognized(self, *_args):
        """Callback method to be called every time speech is recognized."""
        self.memoryProxy.unsubscribeToEvent("WordRecognized", "react")

        # Get WordRecognized data from memory:
        speech_data = self.memoryProxy.getData("WordRecognized")
        # The WordRecognized key is organized as
        # [ph_1, pb_1, ..., ph_n, pb_n], where ph_i is one phrase from
        # the vocabulary and pb_i is the estimated probability that ph_i
        # has been said by the human. The order inside the vector is
        # from the most likely phrases to the less likely.
        # Here, if the person says "human", the robot says "robot":
        if (speech_data[0] == "human" and
                speech_data[1] > self.word_probability):
            self.TTSProxy.say("robot")

        self.memoryProxy.subscribeToEvent("WordRecognized", "react",
                                          "when_speech_recognized")

    def stop_all(self):
        """Stop the recognition of the events."""
        self.memoryProxy.unsubscribeToEvent("PeoplePerception/JustArrived",
                                            "react")
        self.memoryProxy.unsubscribeToEvent("PeoplePerception/JustLeft",
                                            "react")
        self.memoryProxy.unsubscribeToEvent("WordRecognized", "react")
        self.speechRecognitionProxy.pause(True)


def main(args):
    # A broker is an object that allows you to find modules and methods
    # and to call attached methods from outside the process.
    # We need this broker to be able to construct NAOqi modules and
    # subscribe to other modules. The broker must stay alive while the
    # program exists. Keeping the broker alive allows you to create
    # proxies without specifying an IP or port.
    myBroker = ALBroker("myBroker",
                        "0.0.0.0",  # listen to anyone
                        0,  # find a free port and use it
                        args.robot_ip,  # parent broker IP
                        args.port)  # parent broker port

    # The object of the module created *MUST* be a global variable and
    # the same name *MUST* be passed as parameter for the class
    # constructor.
    global react
    react = reactModule("react")

    # Wake up the robot
    motionProxy = ALProxy("ALMotion", args.robot_ip, args.port)
    motionProxy.wakeUp()

    try:
        while True:  # keeping broker alive
            time.sleep(1)
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down")
        react.stop_all()
        myBroker.shutdown()
        sys.exit(0)
    except Exception as e:
        print(e)


if __name__ == "__main__":
    # Command line arguments:
    parser = argparse.ArgumentParser(
        description='First steps programming NAO. Check the tutorial on '
                    'https://github.com/Adorno-Lab/Nao-robot/wiki/'
                    'First-steps-programming-NAO-(Python-and-Ubuntu)')
    parser.add_argument('robot_ip', type=str,
                        help='the IP address of the robot')
    parser.add_argument(
        '-port', type=str, default=9559,
        help='the port on which NAOqi listens (9559 by default)')
    args = parser.parse_args()

    main(args)