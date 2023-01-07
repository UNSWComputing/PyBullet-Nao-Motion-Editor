# move head
# PyBullet Keyboard Shortcuts https://github.com/Gabo-Tor/pybullet-keyboard-shortcuts

from datetime import timedelta
import sys, time
import math
import pybullet as pb
import pybullet_data as pd
#import readchar
from qibullet import SimulationManager
from qibullet import NaoVirtual
# from UtilityFuncs import Lerp
from MotionHandle import MotionHandle as MH
from MotionPlayer import MotionPlayer as MP

PI = 3.1415926535
DEFAULT_POSE = [0, -10, 90, 10, 0, 0, 0, 0, 0, -28, 50, -25, 0, 0, -28, 50, -25, 0, 90, -10, 0, 0, 0, 0, 0]

if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    pb.setAdditionalSearchPath(pd.getDataPath())
    # pb.resetDebugVisualizerCamera( cameraDistance=0.5, cameraYaw=65, cameraPitch=-25, cameraTargetPosition=[0,0,0.35])
    pb.resetDebugVisualizerCamera( cameraDistance=5.0, cameraYaw=0, cameraPitch=-25, cameraTargetPosition=[0,0,0.35])
    
    # Creates better meshes for goalposts. Uses Separating Axis Theorem(SAT) based convex
    # collision detection (instead of GJK and EPA)
    pb.setPhysicsEngineParameter(enableSAT=1)
    pb.setRealTimeSimulation(1)
    
    simulation_manager.setGravity(client, [0.0, 0.0, -9.81])
    
    pb.loadURDF("models/spl-22-field.urdf", 
                basePosition=[0,0,0],
                baseOrientation=pb.getQuaternionFromEuler([PI/2,0,0]),
                useFixedBase=1,
                flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL, 
                physicsClientId=client)

    # Currently, the goalpost uses a convex mesh, i.e. objects can't pass the goal line.
    # Might need to break the goalpost into 2 or more convex parts and combine them in urdf.
    pb.loadURDF("models/spl-goalpost.urdf", 
                basePosition=[-4.5,0,0.05],
                baseOrientation=pb.getQuaternionFromEuler([PI/2,0,0]),
                useFixedBase=1,
                flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                #pb.GEOM_FORCE_CONCAVE_TRIMESH | 
                #pb.GEOM_CONCAVE_INTERNAL_EDGE,
                physicsClientId=client)

    pb.loadURDF("models/spl-goalpost.urdf", 
                basePosition=[4.5,0,0.05],
                baseOrientation=pb.getQuaternionFromEuler([PI/2,0,PI]),
                useFixedBase=1,
                flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                #pb.URDF_INITIALIZE_SAT_FEATURES |
                #pb.GEOM_FORCE_CONCAVE_TRIMESH | 
                #pb.GEOM_CONCAVE_INTERNAL_EDGE, 
                physicsClientId=client)

    # This is from pybullet
    ball = pb.loadURDF("soccerball.urdf",[0,1,1], globalScaling=0.1)
    pb.changeDynamics(ball,-1,linearDamping=0, angularDamping=0, rollingFriction=0.001, spinningFriction=0.001)
    pb.changeVisualShape(ball,-1,rgbaColor=[0.8,0.8,0.8,1])

    # This was made by me
    # pb.loadURDF("models/spl-22-ball.urdf",
    #             basePosition=[0.3,0,0.3],
    #             baseOrientation=pb.getQuaternionFromEuler([0,0,0]),
    #             flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL, 
    #             physicsClientId=client)

    robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)

    # pb.changeDynamics(robot.robot_model, -1, mass=0) # Sets the mass of the base link(-1) to 0.
    
    pb.resetBasePositionAndOrientation(robot.robot_model, [0.0, 0.0, 0.38], pb.getQuaternionFromEuler([0.0, 0.0, 0.0]))

    time.sleep(1.0)
    joint_parameters = list()

    motion_handle1 = MH()
    motion_handle2 = MH()
    # motion_handle1.readPosFile("pos/test-r-arm.pos")
    # motion_handle1.readPosFile("pos/stand.pos")
    motion_handle2.readPosFile("pos/sit.pos")
    # motion_handle1.readPosFile("pos/getupFront.pos")
    # motion_handle.readPosFile("pos/getupBack.pos")
    # motion_handle.readPosFile("pos/ukemiBack.pos")
    # motion_handle.readPosFile("pos/sample_motion-1.pos")
    motion_player = MP(robot, motion_handle2)
    # MP.loop = True

    sit_button = pb.addUserDebugParameter("Sit", 1, 0, 0)
    stand_button = pb.addUserDebugParameter("Stand", 1, 0, 0)
    # robot.goToPosture()
    # for name, joint in robot.joint_dict.items():
    #     print(f"{name:14} | {joint.getLowerLimit():12} | {joint.getUpperLimit():12} | {robot.getAnglesPosition(name):6}")

    try:
        # yaw_angle = 0.0
        # Hyaw_upperlimit = robot.joint_dict.get("HeadYaw").getUpperLimit()
        # Hyaw_lowerlimit = robot.joint_dict.get("HeadYaw").getLowerLimit()
        # Hpitch_upperlimit = robot.joint_dict.get("HeadPitch").getUpperLimit()
        # Hpitch_lowerlimit = robot.joint_dict.get("HeadPitch").getLowerLimit()
        # direction = 1 # 1, -1 -> L, R
        # step = 0.2
        # delay = 0.5
        # prevTime = 0
        # firstTime = True
        # speed = 0.9
        # print(robot.joint_dict.items())
        x = 0.0
        y = 0.0
        v_x = 0.001
        v_y = 0.001
        key_update = False
        dt = round(1/83.333, 3)
        print(f"dt: {dt}")

        curr_sit_button_state = 0
        curr_stand_button_state = 0
        curr_pose = "Default"
        motion_player.curr_pose = [math.radians(v) for v in DEFAULT_POSE]
        # print(len(motion_player.motion_handle.keyframes))

        for i in range(len(motion_player.motion_handle.keyframes)):
            motion_player.keyframe_index = i
            # motion_player.updateCurrPose()
            motion_player.updateTargetPose()
            # print(f"curr: {motion_player.curr_pose[2]}, target: {math.radians(motion_player.target_pose[2])}")
            intermediates = motion_player.generateIntermediateVals(motion_player.target_duration, dt)
            # print("intermediates: ",intermediates)
            print("im #", i)
            for im in intermediates:
                # motion_player.updateCurrPose()
                motion_player.target_pose = im
                print(im)
                motion_player.setJointAngles(1.0)

                currTime = time.time() - motion_player.start_time
                print("currtime: ", currTime)
                if currTime < dt:
                    # print(f"Sleeping for {dt - currTime} sec")
                    time.sleep(dt*1.0 - currTime)
                
                # print("{:.10f}".format(im[0]))
            #print("==[Done]==")
            motion_player.curr_pose = motion_player.target_pose

        # while True:
        #     sit_button_state = pb.readUserDebugParameter(sit_button)
        #     stand_button_state = pb.readUserDebugParameter(stand_button)
        #     # print(f"sit:{sit_button_state}, stand:{stand_button_state}")

        #     if sit_button_state > curr_sit_button_state: 
        #         curr_sit_button_state = sit_button_state
        #         motion_player.motion_handle = motion_handle2
        #         if curr_pose != "SIT":
        #             motion_player.reset()
        #             curr_pose = "SIT"
        #         motion_player.playMotion(0.4)
        #     elif stand_button_state > curr_stand_button_state:
        #         curr_stand_button_state = stand_button_state
        #         motion_player.motion_handle = motion_handle1
        #         if curr_pose != "STAND":
        #             motion_player.reset()
        #             curr_pose = "STAND"
        #         motion_player.playMotion(0.1)
        #     else:
        #         pass
            # if firstTime:
            #     firstTime = False
            #     robot.setAngles("HeadPitch", Hpitch_lowerlimit, speed)
            #     # robot.setAngles("HeadPitch", Hpitch_lowerlimit, speed)
            # key = readchar.readkey() # Can't use this as is since it freezes the simulation
            # if key == "w":
            #     x += v_x
            #     key_update = True
            # if key == "a":
            #     y -= v_y
            #     key_update = True
            # if key == "s":
            #     x -= v_x
            #     key_update = True
            # if key == "d":
            #     y += v_y
            #     key_update = True
            
            # if key_update:
            #     pb.resetBasePositionAndOrientation(robot.robot_model, [x, y, 0.53], pb.getQuaternionFromEuler([0.0, 0.0, 0.0]))
            #     key_update = False
            # curr_angle = robot.getAnglesPosition("HeadPitch")    
            # print("[looping..]")
            # if curr_angle >= Hpitch_upperlimit-0.05:
            #     robot.setAngles("HeadPitch", Hpitch_lowerlimit, speed)
            # elif curr_angle <= Hpitch_lowerlimit+0.05:
            #     robot.setAngles("HeadPitch", Hpitch_upperlimit, speed)
            # simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)
