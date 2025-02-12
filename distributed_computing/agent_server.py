'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import threading
import time

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from kinematics.inverse_kinematics import InverseKinematicsAgent
from jsonrpc import JSONRPCResponseManager, dispatcher
from werkzeug.wrappers import Request, Response
from werkzeug.serving import run_simple


class ServerAgent(InverseKinematicsAgent):
    def __init__(self):
        super(ServerAgent, self).__init__()

        dispatcher["get_angle"] = self.get_angle
        dispatcher["set_angle"] = self.set_angle
        dispatcher["get_posture"] = self.get_posture
        dispatcher["execute_keyframes"] = self.execute_keyframes
        dispatcher["get_transform"] = self.get_transform
        dispatcher["set_transform"] = self.set_transform

    def get_angle(self, joint_name):
        try:
            angle = self.perception.joint[joint_name]
        except:
            return "Error"
        return angle

    def set_angle(self, joint_name, angle):
        try:
            self.perception.joint[joint_name] = angle
        except:
            return "Error"
        return "Successful"

    def get_posture(self):
        posture = self.recognize_posture
        return posture

    def execute_keyframes(self, keyframes):
        agent.keyframes = keyframes
        return "Successful"

    def get_transform(self, name):
        try:
            return self.transforms[name]
        except:
            return "Error"

    def set_transform(self, effector_name, transform):
        try:
            agent.set_transforms(effector_name, transform)
        except:
            return "Error"
        return "Successful"

    @Request.application
    def application(self, request):
        response = JSONRPCResponseManager.handle(request.data, dispatcher)
        return Response(response.json, mimetype='application/json')


if __name__ == '__main__':
    agent = ServerAgent()


    def run_server():
        run_simple('localhost', 4002, agent.application)


    server_thread = threading.Thread(target=run_server).start()

    agent.run()
