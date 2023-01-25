import sys
import time
from datetime import datetime
import math
import pybullet as pb
from qibullet import SimulationManager
from qibullet import NaoVirtual

from MotionHandle import MotionHandle as MH
from UtilityFuncs import ChangeNaoJointOrder

PI = 3.1415926535

if __name__ == "__main__":
    simulation_manager = SimulationManager()

    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    
    simulation_manager.setGravity(client, [0.0, 0.0, -9.81])
    pb.resetDebugVisualizerCamera( cameraDistance=0.5, cameraYaw=65, cameraPitch=-25, cameraTargetPosition=[0,0,0.35])    
    robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)
    
    pb.changeDynamics(robot.robot_model, -1, mass=0) # Sets the mass of the base link(-1) to 0.
    # print(pb.getDynamicsInfo(robot.robot_model, -1))

    time.sleep(1.0)
    joint_parameters = list()

    for name, joint in robot.joint_dict.items():
        if "Finger" not in name and "Thumb" not in name and "RHipYawPitch" not in name:
            joint_parameters.append((
                pb.addUserDebugParameter(
                    "  "+name,
                    joint.getLowerLimit(),
                    joint.getUpperLimit(),
                    robot.getAnglesPosition(name)),
                name))

    rotate_X_slider = pb.addUserDebugParameter("  Rotate X", -PI, PI, 0.0)
    rotate_Y_slider = pb.addUserDebugParameter("  Rotate Y", -PI, PI, 0.0)
    rotate_Z_slider = pb.addUserDebugParameter("  Rotate Z", -PI, PI, 0.0)

    # Not ideal but there doesn't seem to be a better way for now.
    duration_slider = pb.addUserDebugParameter("  Duration(ms)", 50, 5000, 1000)

    mirror_joints_button = pb.addUserDebugParameter("  Mirror Joints", 1, 0, 0)
    reset_joints_button = pb.addUserDebugParameter("  Reset Joints", 1, 0, 0)

    capture_joint_values_button = pb.addUserDebugParameter("  Capture Keyframe", 1, 0, 0)

    try:
        motion_1 = MH()
        button_counter = 1
        reset_joints_button_counter = 1
        while True:
            joint_vals = []
            should_mirror_joints = pb.readUserDebugParameter(mirror_joints_button)

            reset_joints = pb.readUserDebugParameter(reset_joints_button)
            # print(reset_joints, reset_joints_button_counter)

            if reset_joints >= reset_joints_button_counter:
                # Can't set slider positions programatically. So this doesn't quite work yet.
                for joint_parameter in joint_parameters:
                    pb.writeUserDebugParameter(joint_parameter[0], 0.0)
                    robot.setAngles(
                        joint_parameter[1],
                        0.0, 1.0)
                reset_joints_button_counter += 1
            else:
                for joint_parameter in joint_parameters:
                    val = pb.readUserDebugParameter(joint_parameter[0])
                    joint_vals.append(val)
                    
                    if should_mirror_joints % 2 == 1 and joint_parameter[1][0] != 'H':
                        # mirror joints
                        # Joint vals aren't always just negative. Corrections need to be
                        # made for each pair.
                        side = 'L' if joint_parameter[1][0] == 'R' else 'R'
                        robot.setAngles(
                            [joint_parameter[1], (side + joint_parameter[1][1:])],
                            [val, -val], 1.0)
                    else:
                        robot.setAngles(
                            joint_parameter[1],
                            val, 1.0)

            X_rot_val = pb.readUserDebugParameter(rotate_X_slider)
            Y_rot_val = pb.readUserDebugParameter(rotate_Y_slider)
            Z_rot_val = pb.readUserDebugParameter(rotate_Z_slider)
            duration_val = pb.readUserDebugParameter(duration_slider)

            capture_current_pos = pb.readUserDebugParameter(capture_joint_values_button)

            # pb.resetBasePositionAndOrientation(robot.robot_model, [0.0, 0.0, 0.4], pb.getQuaternionFromEuler([X_rot_val, Y_rot_val, Z_rot_val]))
            
            if capture_current_pos >= button_counter:
                motion_1.addKeyFrame(int(duration_val), ChangeNaoJointOrder([math.degrees(j) for j in joint_vals]), description="test descr.")
                button_counter += 1
                # TODO
                # - Find an intuitive way to add the duration and description fields to the GUI.
                # - Might be related to creating a better GUI in general to enable functionality like:
                #   - seeing the keyframes
                #   - deleting, reordering and other manipulations
                #   - visualizing the previous pose as a ghost (nice to have)
            
            simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        timestamp = datetime.now().strftime("%H-%M-%S_%d-%m-%Y")
        simulation_manager.stopSimulation(client)
        motion_1.generatePosFile("motion_"+timestamp)
