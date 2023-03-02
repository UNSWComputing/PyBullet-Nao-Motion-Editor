# Camera Test

from datetime import timedelta
import sys, time
import math
import pybullet as pb
import pybullet_data as pd
import cv2
from qibullet import SimulationManager
from qibullet import NaoVirtual
from qibullet import Camera
# from UtilityFuncs import Lerp
from MotionHandle import MotionHandle as MH
from MotionPlayer import MotionPlayer as MP
from SPLAssets import LoadSPLField, LoadSPLGoalPosts, SpawnSPLBall

DEFAULT_POSE = [0, -10, 90, 10, 0, 0, 0, 0, 0, -28, 50, -25, 0, 0, -28, 50, -25, 0, 90, -10, 0, 0, 0, 0, 0]
USE_GRAVITY = True

if __name__ == "__main__":
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True, auto_step=False)
    pb.setAdditionalSearchPath(pd.getDataPath())
    # pb.resetDebugVisualizerCamera( cameraDistance=0.5, cameraYaw=65, cameraPitch=-25, cameraTargetPosition=[0,0,0.35])
    pb.resetDebugVisualizerCamera( cameraDistance=5.0, cameraYaw=0, cameraPitch=-25, cameraTargetPosition=[0,0,0.35])
    
    # Creates better meshes for goalposts. Uses Separating Axis Theorem(SAT) based convex
    # collision detection (instead of GJK and EPA)
    pb.setPhysicsEngineParameter(enableSAT=1)
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

    handle_top = robot.subscribeCamera(
        NaoVirtual.ID_CAMERA_TOP, 
        resolution=Camera.K_QVGA,
        fps=30)

    handle_bottom = robot.subscribeCamera(
        NaoVirtual.ID_CAMERA_BOTTOM, 
        resolution=Camera.K_QVGA,
        fps=30)

    try:
        while True:
            img_top = robot.getCameraFrame(handle_top)
            img_bottom = robot.getCameraFrame(handle_bottom)
            cv2.imshow("top camera", img_top)
            cv2.imshow("bottom camera", img_bottom)
            cv2.waitKey(1)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)
