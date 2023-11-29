'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it pjsonrpcclientrovides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref

import jsonrpcclient
from jsonrpcclient import request
import sys
sys.path.append('/programming-humanoid-robot-in-python/joint_control/keyframes')
#from keyframes import *

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        response = request("http://localhost:8000", "get_angle", joint_name)
        return response

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        response = request("http://localhost:8000", "set_angle", joint_name=joint_name, angle=angle)
        return response

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        response = request("http://localhost:8000", "get_posture")
        return response

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        response = request("http://localhost:8000", "execute_keyframes", keyframes=keyframes)
        return response

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        response = request("http://localhost:8000", "get_transform", name=name)
        return response

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        response = request("http://localhost:8000", "set_transform", effector_name=effector_name, transform=transform)
        return response

if __name__ == '__main__':
    agent = ClientAgent()

    # TEST CODE HERE
    joint_name = "HeadYaw"
    angle = 45.0
    print(agent.get_angle(joint_name))
    '''
    print(agent.set_angle(joint_name, angle))

    keyframes = hello()  # Provide the actual keyframes
    print(agent.execute_keyframes(keyframes))
    '''


