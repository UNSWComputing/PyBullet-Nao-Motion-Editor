import sys
import time
import math
import pybullet as pb
from qibullet import SimulationManager
from qibullet import NaoVirtual

from MotionGenerator import MotionGenerator as MG
from UtilityFuncs import ChangeNaoJointOrder

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
        if "Finger" not in name and "Thumb" not in name and "RHipYawPitch" not in name:
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

    capture_joint_values_button = pb.addUserDebugParameter("Capture Keyframe", 1, 0, 0)

    try:
        motion_1 = MG()
        button_counter = 1
        while True:
            joint_vals = []
            for joint_parameter in joint_parameters:
                val = pb.readUserDebugParameter(joint_parameter[0])
                joint_vals.append(val)
                robot.setAngles(
                    joint_parameter[1],
                    val, 1.0)

            X_rot_val = pb.readUserDebugParameter(rotate_X_slider)
            Y_rot_val = pb.readUserDebugParameter(rotate_Y_slider)
            Z_rot_val = pb.readUserDebugParameter(rotate_Z_slider)

            capture_current_pos = pb.readUserDebugParameter(capture_joint_values_button)

            pb.resetBasePositionAndOrientation(robot.robot_model, [0.0, 0.0, 0.4], pb.getQuaternionFromEuler([X_rot_val, Y_rot_val, Z_rot_val]))
            
            if capture_current_pos >= button_counter:
                motion_1.addKeyframe(1000, ChangeNaoJointOrder([math.degrees(j) for j in joint_vals]), description="test descr.")
                button_counter += 1
            
            simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        motion_1.generatePosFile("test-motion-2")
        simulation_manager.stopSimulation(client)
