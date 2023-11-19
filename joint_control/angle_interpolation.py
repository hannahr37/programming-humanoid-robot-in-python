'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from time import sleep

import numpy as np
from pid import PIDAgent
from keyframes import hello
import bezier 


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        names, times, keys = keyframes
        
        for k, v in self.perception.joint.items():
            target_joints[k] = 0.

        #print(target_joints)
        
        now = self.perception.time
        duration = max(max(times))
        
        dt = now % duration
        #print(dt)
        for joint_index in range(len(names)):
            joint_name = names[joint_index]
            joint_times = times[joint_index]
            joint_keys = keys[joint_index]

            for times_index in range(len(joint_times)):
                #print(times_index)
                #print(len(joint_times))

                if len(joint_times) <= times_index +1:
                    #print("weiter")
                    continue
                #print(joint_times[times_index])
                if (joint_times[times_index]) <= dt <= (joint_times[times_index+1]):
                    #print("j")

                
                    P0 = [joint_times[times_index], joint_keys[times_index][0]]
                    P1 = [joint_times[times_index]+ joint_keys[times_index][2][1], joint_keys[times_index][0]+ joint_keys[times_index][2][2]]
                    P2 = [joint_times[times_index+1] + joint_keys[times_index+1][1][1], joint_keys[times_index+1][0]+ joint_keys[times_index+1][1][2]]
                    P3 = [joint_times[times_index+1], joint_keys[times_index+1][0]]

                    t = (dt - joint_times[times_index])/(joint_times[times_index+1]-joint_times[times_index])
                    interpolated_angles = self.bezier_interpolation2(t, P0, P1, P2, P3)

                    target_joints[joint_name] = interpolated_angles    
                    #print(str(joint_name) + str(target_joints[joint_name]))
        #print(target_joints)

                
        return target_joints
    
    def bezier_interpolation2(self, i, P0, P1, P2, P3):
        # Cubic Bezier interpolation formula
        return ((1 - i)**3 * P0[1] + 3 * (1 - i)**2 * i * P1[1] + 3 * (1 - i) * i**2 * P2[1] + i**3 * P3[1])
        
    def bezier_interpolation(self, i, P0, P1, P2, P3):
        # Cubic Bezier interpolation formula
        return [
            (1 - i)**3 * P0[0] + 3 * (1 - i)**2 * i * P1[0] + 3 * (1 - i) * i**2 * P2[0] + i**3 * P3[0],
            (1 - i)**3 * P0[1] + 3 * (1 - i)**2 * i * P1[1] + 3 * (1 - i) * i**2 * P2[1] + i**3 * P3[1]
        ]

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
