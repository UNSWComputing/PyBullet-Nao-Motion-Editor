# move head
# PyBullet Keyboard Shortcuts https://github.com/Gabo-Tor/pybullet-keyboard-shortcuts

# Notes:
# - The force for the setAngles method in RobotVirtual was tweaked
#   by adding a multiplier(defualt should be 1.0, currently 8.0).
# - The motion playing is still not very smooth. Needs fixing.

from datetime import timedelta
import sys, time
import math
import pybullet as pb
import pybullet_data as pd
#import readchar
from qibullet import SimulationManager
from qibullet import NaoVirtual, Camera
import cv2
# from UtilityFuncs import Lerp
from MotionHandle import MotionHandle as MH
from MotionPlayer import MotionPlayer as MP
from SPLAssets import LoadSPLField, LoadSPLGoalPosts, SpawnSPLBall

PI = 3.1415926535
DEFAULT_POSE = [0, -10, 90, 10, 0, 0, 0, 0, 0, -28, 50, -25, 0, 0, -28, 50, -25, 0, 90, -10, 0, 0, 0, 0, 0]
USE_GRAVITY = True
USE_ROBOT_CAMERAS = False

if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    pb.setAdditionalSearchPath(pd.getDataPath())
    # pb.resetDebugVisualizerCamera( cameraDistance=0.5, cameraYaw=65, cameraPitch=-25, cameraTargetPosition=[0,0,0.35])
    pb.resetDebugVisualizerCamera( cameraDistance=5.0, cameraYaw=0, cameraPitch=-25, cameraTargetPosition=[0,0,0.35])
    
    # Creates better meshes for goalposts. Uses Separating Axis Theorem(SAT) based convex
    # collision detection (instead of GJK and EPA)
    pb.setPhysicsEngineParameter(enableSAT=1)
    # pb.setTimeStep(1/100) # The performance of motions depends a lot on this.
    pb.setRealTimeSimulation(1)
    
    simulation_manager.setGravity(client, [0.0, 0.0, -9.81])
    
    LoadSPLField(client)
    LoadSPLGoalPosts(client)
    SpawnSPLBall(client, position=[0, 1, 1])
    

    robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)

    if USE_GRAVITY:
        pb.changeDynamics(robot.robot_model, -1, mass=0.01) # Sets the mass of the base link(-1) to 0.
        pb.resetBasePositionAndOrientation(robot.robot_model, [0.0, 0.0, 0.38], pb.getQuaternionFromEuler([0.0, 0.0, 0.0]))
    else:
        pb.changeDynamics(robot.robot_model, -1, mass=0) # Sets the mass of the base link(-1) to 0.
        pb.resetBasePositionAndOrientation(robot.robot_model, [0.0, 0.0, 0.58], pb.getQuaternionFromEuler([0.0, 0.0, 0.0]))
    

    time.sleep(1.0)
    joint_parameters = list()

    if USE_ROBOT_CAMERAS:
        # Set up cameras
        handle_top = robot.subscribeCamera(
            NaoVirtual.ID_CAMERA_TOP, 
            resolution=Camera.K_QVGA,
            fps=15)

        handle_bottom = robot.subscribeCamera(
            NaoVirtual.ID_CAMERA_BOTTOM, 
            resolution=Camera.K_QVGA,
            fps=15)

    motion_handle1 = MH()
    motion_handle2 = MH()

    # Sit and the fast front getup works.
    # In general, motions do seem to get a bit stuck
    # at times either due to contact with a solid such
    # as the floor or the robot itself, or probably because
    # waiting between keyframes should be done based on simulation 
    # time instead of real time ?
    # Or it could be something to do with the masses ? (less likely)
    # 

    # motion_handle1.readPosFile("pos/test-r-arm.pos")
    # motion_handle1.readPosFile("pos/stand.pos") # WORKS
    # motion_handle1.readPosFile("pos/sit.pos") # WORKS
    # motion_handle1.readPosFile("pos/getupFront.pos")
    motion_handle1.readPosFile("pos/Forwards.pos")
    # motion_handle1.readPosFile("pos/Pump_left_arm.pos")
    # motion_handle1.readPosFile("pos/getupFrontFast.pos") # WORKS
    # motion_handle1.readPosFile("pos/getupBackFastV6.pos")
    # motion_handle1.readPosFile("pos/getupBack.pos")
    # motion_handle1.readPosFile("pos/ukemiBack.pos")
    # motion_handle1.readPosFile("pos/sample_motion-1.pos")
    # motion_handle1.readPosFile("pos/pump-arms-up.pos") # WORKS
    # motion_handle1.readPosFile("pos/jump_1.pos")
    motion_player = MP(robot, motion_handle1)
    motion_player.loop = False
    
    # for name, joint in robot.joint_dict.items():
    #     print(f"{name:14} | {joint.getLowerLimit():12} | {joint.getUpperLimit():12} | {robot.getAnglesPosition(name):6}")

    try:
        # The round() is kept in case dt is changed to some other value.
        dt = round(1/30.0, 6) # Keep this at 1/30 -> works for without camera
        # dt = round(1/25.0, 3)
        # print(f"dt: {dt}")
        
        motion_player.curr_pose = [math.radians(v) for v in DEFAULT_POSE]
        
        while True:
            # Get Camera Feed
            if USE_ROBOT_CAMERAS:
                img_top = robot.getCameraFrame(handle_top)
                img_bottom = robot.getCameraFrame(handle_bottom)
                cv2.imshow("bottom camera", img_bottom)
                cv2.imshow("top camera", img_top)
                cv2.waitKey(1)

            motion_player.stepMotion(dt)

            # simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)
