'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


import math
from time import sleep

import numpy as np
from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def rotation(self, axis, theta):
        
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        if axis == 'x':
            T = np.array([
            [1, 0, 0, 0],
            [0, cos_theta, sin_theta, 0],
            [0, sin_theta, cos_theta, 0],
            [0, 0, 0, 1]
            ])
            #print("Roll")
            
        elif axis == 'y':
            T = np.array([
            [cos_theta, 0, sin_theta, 0],
            [0, 1, 0, 0],
            [-sin_theta, 0, cos_theta, 0],
            [0, 0, 0, 1]
            ])
            #print("Pitch")
            
        elif axis == 'z':
            T = np.array([
            [cos_theta, sin_theta, 0, 0],
            [sin_theta, cos_theta, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])
            #print("Yaw")
        
        return T
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        if "LLeg" in effector_name:
            sign = 0-1
        elif "RLeg" in effector_name:
            sign = 1

        l_upperleg = 0.1
        l_lowerleg = 0.1029
        T = identity(4)
        Torso2Leg = transform
        Leg2Torso = np.linalg.inv(transform)
        print("Leg2Torso" + str(Leg2Torso))
        
        Leg2Foot = T
        Leg2Foot[-1,2] += 0.04519
        print("Leg2Foot" + str(Leg2Foot))

        #Foot2Torso = T
        Foot2Torso = np.dot( np.linalg.inv(Leg2Foot), Leg2Torso)
        print("Foot2Torso" + str(Foot2Torso))

        Hip2Torso = T
        Hip2Torso[-1, 1] = (sign*0.05) #anderes Vorzeichen bei Rechts
        Hip2Torso[-1, 2] = 0.085
        print("Hip2Torso" + str(Hip2Torso))

        Foot2Hip = np.dot(Foot2Torso, np.linalg.inv(Hip2Torso))
        print("Foot2Hip" + str(Foot2Hip))
        
        #8.2
        Foot2HipOrthogonal = np.dot(self.rotation('x', np.radians(45)), Foot2Hip)
        print("F2HO" + str(Foot2HipOrthogonal))
        #8.3
        #HipOrthogonal2Foot = np.linalg.inv(Foot2HipOrthogonal)
        HipOrthogonal2Foot = np.dot(np.linalg.inv(Foot2Hip), self.rotation('x', -np.pi/4))
        print(HipOrthogonal2Foot)
        translation_vector = HipOrthogonal2Foot[-1, :3]
        l_trans = np.linalg.norm(translation_vector)
        print(translation_vector)
        #l_trans = np.sqrt(translation_vector[0] ** 2+ translation_vector[1]**2 + translation_vector[3]**2)
        print(l_trans)
        
        gamma = np.arccos((l_upperleg**2 + l_lowerleg**2 - l_trans**2)/(2*l_upperleg*l_lowerleg))
        delta_knee = np.pi - gamma
        #print(delta_knee)
        print((l_lowerleg**2 + l_trans**2 - l_upperleg)/(2*l_lowerleg*l_trans))

        delta_footPitch1 = np.arccos((l_lowerleg**2 + l_trans**2 - l_upperleg**2)/(2*l_lowerleg*l_trans))

        x = Foot2HipOrthogonal [-1, 0]
        y = Foot2HipOrthogonal [-1, 1]
        z = Foot2HipOrthogonal [-1, 2]
        delta_footPitch2 = math.atan2(x, np.sqrt(y**2 + z**2))
        delta_footRoll = math.atan2(y,z)

        delta_footPitch = delta_footPitch1 + delta_footPitch2

        Trans_lowerLeg = T
        Trans_upperLeg = T
        Trans_lowerLeg[-1, 2] = l_lowerleg
        Trans_upperLeg[-1, 2] = l_upperleg


        #Thigh2Foot = np.dot(self.rotation('x', delta_footRoll), self.rotation('y', delta_footPitch), Trans_lowerLeg ,self.rotation('y', delta_knee), Trans_upperLeg)
        Thigh2Foot = np.dot(self.rotation('x', delta_footRoll), np.dot(self.rotation('y', delta_footPitch), np.dot(Trans_lowerLeg, np.dot(self.rotation('y', delta_knee), Trans_upperLeg))))
        HipOrthogonal2Thigh = np.dot(np.linalg.inv(Thigh2Foot), HipOrthogonal2Foot)
        Rot_Hip = HipOrthogonal2Thigh
        print("Shape" + str(Rot_Hip.shape))
        print(Rot_Hip[2, 1])
        delta_x = np.arcsin(Rot_Hip[2, 1])
        delta_hipYaw = np.arctan2(-Rot_Hip[0,1], Rot_Hip[1,1])
        delta_hipPitch = np.arctan2(-Rot_Hip[2,0], Rot_Hip[2,2])
        delta_hipRoll = delta_x - np.pi/4

        joint_angles = [delta_hipYaw, delta_hipRoll, delta_hipPitch, delta_knee, delta_footPitch, delta_footRoll]
        joint_names = ['HipYaw', 'HipRoll', 'HipPitch', 'Knee', 'FootPitch', 'FootRoll']

        for name, angle in zip(joint_names, joint_angles):
            print(f'{name}: {np.degrees(angle):.2f} degrees')
        #HipPitch Winkel zu groß!
        
        
        return joint_angles
    
    

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        if "LLeg" in effector_name:
            names = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
        elif "RLeg" in effector_name:
            names = ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
        angles = []
        for e in self.inverse_kinematics(effector_name, transform):
            angles.append([e])

        #self.keyframes = ([], [], [])  # the result joint angles have to fill in
        times = [[1.5], [2.0], [2.5], [3.0], [3.5], [4.0]]
        self.keyframes = (names, times, angles)
        print(self.keyframes)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    print("inverse")
    print(T)
    #sleep(3)
    agent.set_transforms('LLeg', T)
    agent.run()
