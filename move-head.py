# move head

from datetime import timedelta
import sys, time
import pybullet as pb
import pybullet_data as pd
from qibullet import SimulationManager
from qibullet import NaoVirtual
# from UtilityFuncs import Lerp
from MotionHandle import MotionHandle as MH
from MotionPlayer import MotionPlayer as MP

PI = 3.1415926535

if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    pb.setAdditionalSearchPath(pd.getDataPath())
    pb.resetDebugVisualizerCamera( cameraDistance=0.5, cameraYaw=65, cameraPitch=-25, cameraTargetPosition=[0,0,0.35])
    
    # Creates better meshes for goalposts. Uses Separating Axis Theorem(SAT) based convex
    # collision detection (instead of GJK and EPA)
    pb.setPhysicsEngineParameter(enableSAT=1)
    
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
                basePosition=[-4.5,0,0.04],
                baseOrientation=pb.getQuaternionFromEuler([PI/2,0,0]),
                useFixedBase=1,
                flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                physicsClientId=client)

    pb.loadURDF("models/spl-goalpost.urdf", 
                basePosition=[4.5,0,0.04],
                baseOrientation=pb.getQuaternionFromEuler([PI/2,0,PI]),
                useFixedBase=1,
                flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL | pb.URDF_INITIALIZE_SAT_FEATURES, 
                physicsClientId=client)

    # This is from pybullet
    ball = pb.loadURDF("soccerball.urdf",[0,0,1], globalScaling=0.1)
    pb.changeDynamics(ball,-1,linearDamping=0, angularDamping=0, rollingFriction=0.001, spinningFriction=0.001)
    pb.changeVisualShape(ball,-1,rgbaColor=[0.8,0.8,0.8,1])

    # This was made by me
    # pb.loadURDF("models/spl-22-ball.urdf",
    #             basePosition=[0.3,0,0.3],
    #             baseOrientation=pb.getQuaternionFromEuler([0,0,0]),
    #             flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL, 
    #             physicsClientId=client)

    robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)

    # pb.changeDynamics(robot.robot_model, -1, mass=0.000000001) # Sets the mass of the base link(-1) to 0.
    
    pb.resetBasePositionAndOrientation(robot.robot_model, [0.0, 0.0, 0.53], pb.getQuaternionFromEuler([0.0, 0.0, 0.0]))

    time.sleep(1.0)
    joint_parameters = list()

    motion_handle = MH()
    # motion_handle.readPosFile("pos/sit.pos")
    # motion_handle.readPosFile("pos/getupFront.pos")
    # motion_handle.readPosFile("pos/getupBack.pos")
    motion_handle.readPosFile("pos/ukemiBack.pos")
    # motion_handle.readPosFile("pos/sample_motion-1.pos")
    motion_player = MP(robot, motion_handle)
    # MP.loop = True

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

        while True:
            motion_player.playMotion()
            # if firstTime:
            #     firstTime = False
            #     robot.setAngles("HeadPitch", Hpitch_lowerlimit, speed)
            #     # robot.setAngles("HeadPitch", Hpitch_lowerlimit, speed)
            
            # curr_angle = robot.getAnglesPosition("HeadPitch")    
            
            # if curr_angle >= Hpitch_upperlimit-0.05:
            #     robot.setAngles("HeadPitch", Hpitch_lowerlimit, speed)
            # elif curr_angle <= Hpitch_lowerlimit+0.05:
            #     robot.setAngles("HeadPitch", Hpitch_upperlimit, speed)
            simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)
