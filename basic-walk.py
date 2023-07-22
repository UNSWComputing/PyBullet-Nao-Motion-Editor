# move head

from datetime import timedelta
import sys, time
from tarfile import LENGTH_PREFIX
from traceback import walk_stack
import pybullet as pb
from qibullet import SimulationManager
from qibullet import NaoVirtual
# from UtilityFuncs import Lerp


if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    
    simulation_manager.setGravity(client, [0.0, 0.0, -9.8])
    
    robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)

    # pb.changeDynamics(robot.robot_model, -1, mass=0) # Sets the mass of the base link(-1) to 0.

    time.sleep(1.0)
    joint_parameters = list()

    # robot.goToPosture()
    # for name, joint in robot.joint_dict.items():
    #     print(f"{name:14} | {joint.getLowerLimit():12} | {joint.getUpperLimit():12} | {robot.getAnglesPosition(name):6}")

    try:
        yaw_angle = 0.0
        LSP_upperlimit = robot.joint_dict.get("LShoulderPitch").getUpperLimit()
        LSP_lowerlimit = 0.9
        RSP_upperlimit = robot.joint_dict.get("RShoulderPitch").getUpperLimit()
        RSP_lowerlimit = 0.9
        LER_upperlimit = robot.joint_dict.get("LElbowRoll").getUpperLimit()
        RER_lowerlimit = robot.joint_dict.get("RElbowRoll").getLowerLimit()


        direction = 1 # 1, -1 -> L, R
        step = 0.2
        delay = 1.5
        prev_time = 0.0
        curr_time = 0.0
        firstTime = True
        speed = 0.01 # Might be stiffness ?

        walk_switch = True
        

        while True:
            # robot.goToPosture("Stand", 0.6)
            if firstTime:
                firstTime = False
                # Bring arms out a bit
                robot.setAngles("LShoulderRoll", 0.168, speed)
                robot.setAngles("RShoulderRoll", -0.168, speed)

                robot.setAngles("LElbowRoll", LER_upperlimit, speed)
                robot.setAngles("RElbowRoll", RER_lowerlimit, speed)
                
                # robot.setAngles("LShoulderPitch", LSP_lowerlimit, speed)
                # robot.setAngles("RShoulderPitch", RSP_upperlimit, speed)
            
            curr_angle = robot.getAnglesPosition("HeadPitch")    
            curr_time = time.time()
            if curr_time > (prev_time + delay):
                prev_time = curr_time
                
                if walk_switch:
                    robot.setAngles("LShoulderPitch", LSP_lowerlimit, speed)
                    robot.setAngles("RShoulderPitch", RSP_upperlimit, 0.2)
                    # Legs
                    robot.setAngles("LHipPitch", 0.1, speed)
                    robot.setAngles("RHipPitch", -0.1, speed)
                    walk_switch = False
                else:
                    robot.setAngles("LShoulderPitch", LSP_upperlimit, speed)
                    robot.setAngles("RShoulderPitch", RSP_lowerlimit, 0.2)
                    # Legs
                    robot.setAngles("LHipPitch", -0.1, speed)
                    robot.setAngles("RHipPitch", 0.1, speed)
                    walk_switch = True
            
            simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)
