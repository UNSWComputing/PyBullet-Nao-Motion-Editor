# move head

from datetime import timedelta
import sys, time
import pybullet as pb
from qibullet import SimulationManager
from qibullet import NaoVirtual
from UtilityFuncs import Lerp


if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    
    simulation_manager.setGravity(client, [0.0, 0.0, 0.0])
    
    robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)

    time.sleep(1.0)
    joint_parameters = list()

    # robot.goToPosture()
    # for name, joint in robot.joint_dict.items():
    #     print(f"{name:14} | {joint.getLowerLimit():12} | {joint.getUpperLimit():12} | {robot.getAnglesPosition(name):6}")

    try:
        yaw_angle = 0.0
        upperlimit = robot.joint_dict.get("HeadYaw").getUpperLimit()
        lowerlimit = robot.joint_dict.get("HeadYaw").getLowerLimit()
        direction = 1 # 1, -1 -> L, R
        step = 0.2
        delay = 0.5
        prevTime = 0
        firstTime = True
        speed = 0.1
        while True:
            if firstTime:
                firstTime = False
                robot.setAngles("HeadYaw", lowerlimit, speed)
            curr_angle = robot.getAnglesPosition("HeadYaw")
            if curr_angle >= upperlimit-0.05:
                robot.setAngles("HeadYaw", lowerlimit, speed)
            elif curr_angle <= lowerlimit+0.05:
                robot.setAngles("HeadYaw", upperlimit, speed)
            # simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)
