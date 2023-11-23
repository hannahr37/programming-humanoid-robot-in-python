'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
import numpy as np
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'], #, 'LWristYaw2', 'LHand2'
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'], #, 'RWristYaw2', 'RHand2'
                       #'Torso': ['Head', 'LArm', 'LLeg', 'RLeg', 'RArm'] #?
                       # YOUR CODE HERE
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        '''
        
        theta = joint_angle
        axis = axis / np.linalg.norm(axis)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cross_product_matrix = np.array([
                [0, -axis[2], axis[1], 0],
                [axis[2], 0, -axis[0], 0],
                [-axis[1], axis[0], 0, 0],
                [0, 0, 0, 1]
            ])
        rotation_matrix = cos_theta * np.identity(4) + (1 - cos_theta) * np.outer(axis, axis) + sin_theta * cross_product_matrix
        
    
        if "Roll" in joint_name:
            T = rotation_matrix(np.array([1, 0, 0]), theta)
            print("roll")
            
        if "Pitch" in joint_name:
            T = rotation_matrix(np.array([0, 1, 0]), theta)
            print("Pitch")
            
        if "Yaw" in joint_name:
            T = rotation_matrix(np.array([0, 0, 1]), theta)
            print("Yaw")

        '''
        theta = joint_angle
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        if "Roll" in joint_name:
            T = np.array([
            [1, 0, 0, 0],
            [0, cos_theta, sin_theta, 0],
            [0, sin_theta, cos_theta, 0],
            [0, 0, 0, 1]
            ])
            #print("Roll")
            
        elif "Pitch" in joint_name:
            T = np.array([
            [cos_theta, 0, sin_theta, 0],
            [0, 1, 0, 0],
            [-sin_theta, 0, cos_theta, 0],
            [0, 0, 0, 1]
            ])
            #print("Pitch")
            
        elif "Yaw" in joint_name:
            T = np.array([
            [cos_theta, sin_theta, 0, 0],
            [sin_theta, cos_theta, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
            #print("Yaw")
        
        #print(T)

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                #print("Tl" + str(Tl))
                # YOUR CODE HERE
                T = np.dot(T, Tl)
                #print("T")
                #print (T)

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
