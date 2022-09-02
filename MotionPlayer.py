# Motion Player Class

import math

from UtilityFuncs import JointErrors, ChangeNaoJointOrder
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

    def updateCurrPose(self):
        self.curr_pose = []
        for name in self.joint_names:
            self.curr_pose.append(self.robot.getAnglesPosition(name))
    
    def updateTargetPose(self):
        self.target_pose = self.motion_handle.keyframes[self.keyframe_index]["joint_vals"]
        self.target_pose_stiffness = self.motion_handle.keyframes[self.keyframe_index]["stiffness_vals"]
        # self.target_duration = self.motion_handle.keyframes[self.keyframe_index]["duration"]
        self.target_duration = 2000

    def setJointAngles(self):
        print("||[ Setting joint angles ]||... ", self.keyframe_index)
        # print("joint names: ", self.joint_names)
        # print("target pose: ", [math.radians(val) for val in self.target_pose])
        self.robot.setAngles(ChangeNaoJointOrder(self.joint_names, 1), [math.radians(val) for val in self.target_pose], [(val*0.1) for val in self.target_pose_stiffness])
        self.start_time = time.time()

        # Not sure how to approach setting the joint speed.
        # Does the percentage speed used by qibullet correspond to stiffness ? or is that 
        # for TorqueControl MODE only ?
        # Or maybe interpolation needs to be done between the keyframes ?

    
    def playMotion(self):
        self.updateCurrPose()
        # print("len: ", len(self.motion_handle.keyframes))
        # print("curr_pose: ", self.curr_pose)
        if not self.Deactivate:
            self.updateTargetPose()
            error_threshold = 0.3
            
            joint_errors = JointErrors(ChangeNaoJointOrder(self.curr_pose, 1), [math.radians(val) for val in self.target_pose])
            
            if self.debug_counter % 200 == 0:
                # print("joint_errors: ", joint_errors)
                print("joint errors: ", max(joint_errors))
            self.debug_counter += 1
            
            if max(joint_errors) >= error_threshold:
                if not self.isMoving and ((time.time() - self.start_time)*1000.0 >= self.target_duration):
                    self.setJointAngles()
                    self.isMoving = True
            else:
                self.isMoving = False
                self.keyframe_index += 1

                if self.keyframe_index >= len(self.motion_handle.keyframes):
                    if self.loop:
                        self.keyframe_index = 0
                    else:                        
                        self.Deactivate = True
            