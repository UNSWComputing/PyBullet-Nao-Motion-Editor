# move head

from datetime import timedelta
import sys, time
import pybullet as pb
from qibullet import SimulationManager
from qibullet import NaoVirtual
# from UtilityFuncs import Lerp


if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    
    simulation_manager.setGravity(client, [0.0, 0.0, 0.0])
    
    robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)

    pb.changeDynamics(robot.robot_model, -1, mass=0) # Sets the mass of the base link(-1) to 0.

    time.sleep(1.0)
    joint_parameters = list()

    # robot.goToPosture()
    # for name, joint in robot.joint_dict.items():
    #     print(f"{name:14} | {joint.getLowerLimit():12} | {joint.getUpperLimit():12} | {robot.getAnglesPosition(name):6}")

    try:
        yaw_angle = 0.0
        Hyaw_upperlimit = robot.joint_dict.get("HeadYaw").getUpperLimit()
        Hyaw_lowerlimit = robot.joint_dict.get("HeadYaw").getLowerLimit()
        Hpitch_upperlimit = robot.joint_dict.get("HeadPitch").getUpperLimit()
        Hpitch_lowerlimit = robot.joint_dict.get("HeadPitch").getLowerLimit()
        direction = 1 # 1, -1 -> L, R
        step = 0.2
        delay = 0.5
        prevTime = 0
        firstTime = True
        speed = 0.9
        

        while True:
            if firstTime:
                firstTime = False
                robot.setAngles("HeadPitch", Hpitch_lowerlimit, speed)
                # robot.setAngles("HeadPitch", Hpitch_lowerlimit, speed)
            
            curr_angle = robot.getAnglesPosition("HeadPitch")    
            
            if curr_angle >= Hpitch_upperlimit-0.05:
                robot.setAngles("HeadPitch", Hpitch_lowerlimit, speed)
            elif curr_angle <= Hpitch_lowerlimit+0.05:
                robot.setAngles("HeadPitch", Hpitch_upperlimit, speed)
            simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)

# GLSL version 1.5 not supported. (Happens sometimes on VirtualBox VM)
# See https://github.com/bulletphysics/bullet3/issues/1681
# Usually fixed by disabling 3D acceleration in VM settings.