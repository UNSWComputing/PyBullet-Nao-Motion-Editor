# move head

from datetime import timedelta
import sys, time
import pybullet as pb
from qibullet import SimulationManager
from qibullet import NaoVirtual
# from UtilityFuncs import Lerp
from MotionHandle import MotionHandle as MH
from MotionPlayer import MotionPlayer as MP


if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    
    pb.resetDebugVisualizerCamera( cameraDistance=0.5, cameraYaw=65, cameraPitch=-25, cameraTargetPosition=[0,0,0.35])
    simulation_manager.setGravity(client, [0.0, 0.0, -9.81])
    
    robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)

    pb.changeDynamics(robot.robot_model, -1, mass=0) # Sets the mass of the base link(-1) to 0.
    
    pb.resetBasePositionAndOrientation(robot.robot_model, [0.0, 0.0, 0.5], pb.getQuaternionFromEuler([0.0, 0.0, 0.0]))

    time.sleep(1.0)
    joint_parameters = list()

    motion_handle = MH()
    # motion_handle.readPosFile("sit.pos")
    motion_handle.readPosFile("getupFront.pos")
    # motion_handle.readPosFile("ukemiBack.pos")
    # motion_handle.readPosFile("sample_motion-1.pos")
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
