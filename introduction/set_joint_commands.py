'''
In this exercise you need to know how to set joint commands.

* Tasks:
    1. set stiffness of LShoulderPitch to 0
    2. set speed of HeadYaw to 0.1

* Hint: The commands are stored in action (class Action in spark_agent.py)

'''

# add PYTHONPATH
import os
import sys
from time import sleep
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'software_installation'))

from spark_agent import SparkAgent


class MyAgent(SparkAgent):
    def think(self, perception):
        sleep(3)
        action = super(MyAgent, self).think(perception)
        action.stiffness['LShoulderPitch'] = 0
        print("stiffness: " + str(action.stiffness))
        action.speed['HeadYaw'] = 0.1
        print("speed: " + str(action.speed))
        # YOUR CODE HERE

        return action

if '__main__' == __name__:
    agent = MyAgent()
    agent.run()
