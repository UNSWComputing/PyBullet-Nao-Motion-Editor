import sys
import time
import pybullet as pb
from qibullet import SimulationManager
from qibullet import NaoVirtual

if __name__ == "__main__":
    simulation_manager = SimulationManager()

    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    
    robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)

    time.sleep(1.0)
    joint_parameters = list()

    for name, joint in robot.joint_dict.items():
        if "Finger" not in name and "Thumb" not in name:
            joint_parameters.append((
                pb.addUserDebugParameter(
                    name,
                    joint.getLowerLimit(),
                    joint.getUpperLimit(),
                    robot.getAnglesPosition(name)),
                name))

    try:
        while True:
            for joint_parameter in joint_parameters:
                robot.setAngles(
                    joint_parameter[1],
                    pb.readUserDebugParameter(joint_parameter[0]), 1.0)

            simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)