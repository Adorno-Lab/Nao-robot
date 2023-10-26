Here you will find some example scripts to work with NAO. The scripts have comments to help you understand what is being done and some of them are linked to a tutorial from the [Wiki](https://github.com/Adorno-Lab/Nao-robot/wiki). Brief descriptions are given below.

Remember to always run the scripts using **Python 2.7**

```shell
python2.7 <script>.py <arguments>
```

and you can use the ```-h``` option to obtain more information about each one
```shell
python2.7 <script>.py -h
```

### List of examples
- [first-steps.py](./first-steps.py): it makes the robot speak and walk. Check the tutorial [First steps programming NAO (Python and Ubuntu)](https://github.com/Adorno-Lab/Nao-robot/wiki/First-steps-programming-NAO-(Python-and-Ubuntu)).
- [events.py](./events.py): it makes the robot react to some events. It shows how to create a [Broker](http://doc.aldebaran.com/2-1/dev/naoqi/index.html#naoqi-broker) and keep it alive, how to create a module, and to track and react to events, including making the robot recognize words said by the human. You can find the Aldebaran example of a Python module to react to events [here](http://doc.aldebaran.com/2-1/dev/python/reacting_to_events.html?highlight=broker).
- [wbb_cartesian_control.py](./wbb_cartesian_control.py): cartesian control of the robot's arms and head using the Whole Body Balancer, a tool to generate safe and natural motion. Check the tutorial [Making NAO move](https://github.com/Adorno-Lab/Nao-robot/wiki/Making-NAO-move).
- [cartesian_control.py](./cartesian_control.py): cartesian control of the robot's arms and head. Check the tutorial [Making NAO move](https://github.com/Adorno-Lab/Nao-robot/wiki/Making-NAO-move).
- [joint_control.py](./joint_control.py): joint control of the robot's arms and head. Check the tutorial [Making NAO move](https://github.com/Adorno-Lab/Nao-robot/wiki/Making-NAO-move).
