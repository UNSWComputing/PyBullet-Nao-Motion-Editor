import sys
import time
import pybullet as pb
from qibullet import SimulationManager
from qibullet import NaoVirtual

PI = 3.1415926535

if __name__ == "__main__":
    simulation_manager = SimulationManager()

    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    
    robot = simulation_manager.spawnNao(client, spawn_ground_plane=False)
    
    pb.changeDynamics(robot.robot_model, -1, mass=0) # Sets the mass of the base link(-1) to 0.
    # print(pb.getDynamicsInfo(robot.robot_model, -1))

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

    rotate_X_slider = pb.addUserDebugParameter("Rotate X", -PI, PI, 0.0)
    rotate_Y_slider = pb.addUserDebugParameter("Rotate Y", -PI, PI, 0.0)
    rotate_Z_slider = pb.addUserDebugParameter("Rotate Z", -PI, PI, 0.0)

    try:
        while True:
            for joint_parameter in joint_parameters:
                robot.setAngles(
                    joint_parameter[1],
                    pb.readUserDebugParameter(joint_parameter[0]), 1.0)

            X_rot_val = pb.readUserDebugParameter(rotate_X_slider)
            Y_rot_val = pb.readUserDebugParameter(rotate_Y_slider)
            Z_rot_val = pb.readUserDebugParameter(rotate_Z_slider)
            pb.resetBasePositionAndOrientation(robot.robot_model, [0.0, 0.0, 0.4], pb.getQuaternionFromEuler([X_rot_val, Y_rot_val, Z_rot_val]))
            
            simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)