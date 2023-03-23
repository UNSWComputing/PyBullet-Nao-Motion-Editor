# Motion Player Class

import math

from UtilityFuncs import JointErrors, ChangeNaoJointOrder, Lerp
from Plotter import JointErrorBars
import time
import csv

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
        self.intermediates = {}
        self.intermediate_index = 0
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
        # Why are we converting dur to seconds here ?
        # Because dt is in seconds.
        time_steps = int((dur/1000)//dt) # rounded down, i.e. robot waits for remaining time
        if time_steps <= 0:
            print(f"Error: time_steps={time_steps}")
            print(f"dt={dt}, dur={dur}")
            raise ZeroDivisionError
        intermediate_joint_vals = {}
        # for each joint in target pose
        max_steps = 0
        for i in range(len(self.target_pose)):            
            j_vals = Lerp(self.curr_pose[i], math.radians(self.target_pose[i]), time_steps)
            # print(j_vals)
            # print(f"=== {len(j_vals)} steps ===")
            if not j_vals:
                # print(f"empty j_vals in {self.joint_names[i]}")
                j_vals = [math.radians(self.target_pose[i])]*(time_steps)
            
            intermediate_joint_vals[self.joint_names[i]] = j_vals
            if len(j_vals)>max_steps:
                max_steps = len(j_vals)

        # To fix the weird issue with interpolation where it goes off by one depending on dt
        for k in intermediate_joint_vals.keys():
            if len(intermediate_joint_vals[k]) < max_steps:
                intermediate_joint_vals[k].append(intermediate_joint_vals[k][-1])

        # for k, v in intermediate_joint_vals.items():
        #     print(f"{k:<18}", len(v))
        # print("------")
        # print(len(intermediate_joint_vals))

        # for iv in list(map(list,zip(*intermediate_joint_vals.values()))):
        #     with open("intermed_joint_data.csv", '+a', newline='') as my_csv:
        #         wr = csv.writer(my_csv, quoting=csv.QUOTE_ALL)
        #         wr.writerow(iv)

        return intermediate_joint_vals
    
    def prepMotion(self, dt):
        self.updateTargetPose()
        self.intermediates = self.generateIntermediateVals(self.target_duration, dt)

    def stepMotion(self, dt):
        # While there are still keyframes to reach
        # print(self.keyframe_index)
        if self.keyframe_index < len(self.motion_handle.keyframes):
            # If we've reached a target keyframe
            if not self.intermediates or self.intermediate_index >= max([len(row) for row in self.intermediates.values()]):
                # move on to next one and generate new in-betweens
                self.intermediate_index = 0
                if(self.keyframe_index >= len(self.motion_handle.keyframes)-1):
                    self.keyframe_index = 0
                else:
                    self.keyframe_index += 1
                # self.curr_pose = self.target_pose
                # print("kf", self.keyframe_index)
                self.prepMotion(dt)
            
            # print(f"max={max([len(row) for row in self.intermediates.values()])}")
            # print("before ",self.intermediates.values())
            # print(self.intermediate_index)
            self.target_pose = [ v[self.intermediate_index] for v in self.intermediates.values()]
            
            # with open("joint_data.csv", '+a', newline='') as my_csv:
            #     wr = csv.writer(my_csv, quoting=csv.QUOTE_ALL)
            #     wr.writerow(self.target_pose)
            
            self.setJointAngles(1.0)
            self.intermediate_index += 1
            currTime = time.time() - self.start_time
            # if the move was completed faster than a time step,
            # we wait for the remaining time.
            if currTime < dt:
                time.sleep(dt*1.0 - currTime)
        else:
            self.keyframe_index = 0
            print("Done")

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
            