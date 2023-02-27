# Motion Player Class

import math

from UtilityFuncs import JointErrors, ChangeNaoJointOrder, Lerp
from Plotter import JointErrorBars
import time

class MotionPlayer:
    def __init__(self, robot, motion_handle):
        self.loop = False
        self.start_time = 0.0
        self.robot = robot
        self.motion_handle = motion_handle
        self.keyframe_index = 0
        self.curr_pose = []
        self.target_pose = []
        self.target_pose_stiffness = []
        self.target_duration = 0 # ms
        self.isMoving = False # whether the robot is currently executing a move
        self.Deactivate = False
        self.debug_counter = 0
        self.joint_names = []
        for name in self.robot.joint_dict.keys():
            if ("Finger" not in name) and \
               ("Thumb" not in name) and  \
               ("RHipYawPitch" not in name):
                self.joint_names.append(name)
        # print(self.joint_names, len(self.joint_names))

    def reset(self):
        self.start_time = 0.0
        self.keyframe_index = 0
        self.curr_pose = []
        self.target_pose = []
        self.target_pose_stiffness = []
        self.target_duration = 0 # ms
        self.isMoving = False # whether the robot is currently executing a move
        self.Deactivate = False
        self.debug_counter = 0

    def updateCurrPose(self):
        self.curr_pose = []
        for name in self.joint_names:
            self.curr_pose.append(self.robot.getAnglesPosition(name))
    
    def updateTargetPose(self):
        self.target_pose = self.motion_handle.keyframes[self.keyframe_index]["joint_vals"]
        self.target_pose_stiffness = self.motion_handle.keyframes[self.keyframe_index]["stiffness_vals"]
        self.target_duration = self.motion_handle.keyframes[self.keyframe_index]["duration"]
        # self.target_duration = 500

    def setJointAngles(self, perc_speed):
        self.start_time = time.time()
        runswift_joint_names = ChangeNaoJointOrder(self.joint_names, 1)
        joint_vals = [val for val in self.target_pose]
        joint_powers = [(perc_speed) for val in self.target_pose_stiffness]
        
        # Explicitly couple HipYawPitch joint
        runswift_joint_names.append("RHipYawPitch")
        joint_vals.append(joint_vals[7])
        joint_powers.append(perc_speed)

        self.robot.setAngles(runswift_joint_names, joint_vals, joint_powers)

        # Not sure how to approach setting the joint speed.
        # Does the percentage speed used by qibullet correspond to stiffness ? or is that 
        # for TorqueControl MODE only ?
        # Or maybe interpolation needs to be done between the keyframes ?

    def generateIntermediateVals(self, dur, dt):
        time_steps = int((dur/1000)//dt) # rounded down, i.e. robot waits for remaining time
        intermediate_joint_vals = []
        for i in range(len(self.target_pose)):            
            j_vals = Lerp(self.curr_pose[i], math.radians(self.target_pose[i]), time_steps)
            if not j_vals:
                j_vals = [math.radians(self.target_pose[i])]*time_steps
            
            intermediate_joint_vals.append(j_vals)
        return list(map(list, zip(*intermediate_joint_vals)))
    
    # Not used currently
    def playMotion(self, effort):
        self.updateCurrPose()

        if not self.Deactivate:
            self.updateTargetPose()
            error_threshold = 2.0
            # Maybe try measuring the fluctuation of the joint error to decide when to switch to 
            # the next motion ? Say if the error remains the same for 500 iterations, consider transitioning ?
            # Not sure what's the best way to do this.
            
            joint_errors = JointErrors(ChangeNaoJointOrder(self.curr_pose, 1), [math.radians(val) for val in self.target_pose])
            
            if self.debug_counter % 200 == 0:
                print("joint errors: ", max(joint_errors))
            self.debug_counter += 1
            
            if not self.isMoving and ((time.time() - self.start_time)*1000.0 >= self.target_duration):
                self.setJointAngles(effort)
                self.isMoving = True
            else:
                self.isMoving = False
                self.keyframe_index += 1
                print(self.keyframe_index)

                if self.keyframe_index >= len(self.motion_handle.keyframes):
                    print("check")
                    if self.loop:
                        self.keyframe_index = 0
                    else:                        
                        self.Deactivate = True
            