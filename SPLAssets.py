# SPL Assets

import pybullet as pb
from math import pi

def LoadSPLField(client):
    pb.loadURDF("models/spl-22-field.urdf", 
                basePosition=[0,0,0],
                baseOrientation=pb.getQuaternionFromEuler([pi/2,0,0]),
                useFixedBase=1,
                flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL, 
                physicsClientId=client)
def LoadSPLGoalPosts(client):    
    # Currently, the goalpost uses a convex mesh, i.e. objects can't pass the goal line.
    # Might need to break the goalpost into 2 or more convex parts and combine them in urdf.
    pb.loadURDF("models/spl-goalpost.urdf", 
                basePosition=[-4.5,0,0.05],
                baseOrientation=pb.getQuaternionFromEuler([pi/2,0,0]),
                useFixedBase=1,
                flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                #pb.GEOM_FORCE_CONCAVE_TRIMESH | 
                #pb.GEOM_CONCAVE_INTERNAL_EDGE,
                physicsClientId=client)

    pb.loadURDF("models/spl-goalpost.urdf", 
                basePosition=[4.5,0,0.05],
                baseOrientation=pb.getQuaternionFromEuler([pi/2,0,pi]),
                useFixedBase=1,
                flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                #pb.URDF_INITIALIZE_SAT_FEATURES |
                #pb.GEOM_FORCE_CONCAVE_TRIMESH | 
                #pb.GEOM_CONCAVE_INTERNAL_EDGE, 
                physicsClientId=client)
    
def SpawnSPLBall(client, position=[0, 0, 1], source="pybullet"):
    if source == "pybullet":
        # This is from pybullet
        ball = pb.loadURDF("soccerball.urdf",[0,1,1], globalScaling=0.1)
        pb.changeDynamics(ball,-1,linearDamping=0, angularDamping=0, rollingFriction=0.001, spinningFriction=0.001)
        pb.changeVisualShape(ball,-1,rgbaColor=[0.8,0.8,0.8,1])
    elif source == "custom":
        # This was made by me
        pb.loadURDF("models/spl-22-ball.urdf",
                    basePosition=[0.3,0,0.3],
                    baseOrientation=pb.getQuaternionFromEuler([0,0,0]),
                    flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL, 
                    physicsClientId=client)
