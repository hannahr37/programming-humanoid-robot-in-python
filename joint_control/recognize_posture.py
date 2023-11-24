'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


import pickle

import numpy as np
from angle_interpolation import AngleInterpolationAgent
from keyframes import hello


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        ROBOT_POSE_CLF = 'joint_control/robot_pose.pkl'
        with open(ROBOT_POSE_CLF, 'rb') as file:
            clf2 = pickle.load(file, encoding='latin1')
        self.posture_classifier = clf2  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        keys = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'AngleX', 'AngleY']
        data = {key: [] for key in keys}

        #print(type(perception))
        pclf = self.posture_classifier

        for k, v in self.perception.joint.items():
            if k in keys:
                data[k] = v
            
        data = np.array(data)
        #print(data)
        #posture = pclf.predict(data.reshape(1, -1))
        #print(perception.__dict__)
        
        posture = 'Stand'
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
