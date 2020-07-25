#!/usr/bin/env python

import pybullet as p
import time
import pybullet_data
import numpy as np

import sys

sys.path.append('../../../')
from spotmicro.util import pybullet_data as pd

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -9.81)
# p.setTimeStep(1./240.)       # slow, accurate
p.setRealTimeSimulation(0)  # we want to be faster than real time :)
planeId = p.loadURDF("plane.urdf")
StartPos = [0, 0, 0.3]
StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
p.resetDebugVisualizerCamera(cameraDistance=0.8,
                             cameraYaw=45,
                             cameraPitch=-30,
                             cameraTargetPosition=[0, 0, 0])
boxId = p.loadURDF(pd.getDataPath() + "/assets/urdf/spot.urdf",
                   StartPos,
                   StartOrientation,
                   flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

numj = p.getNumJoints(boxId)
numb = p.getNumBodies()
Pos, Orn = p.getBasePositionAndOrientation(boxId)
print(Pos, Orn)
print("Number of joints {}".format(numj))
print("Number of links {}".format(numb))
joint = []
movingJoints = [
    6,
    7,
    8,  # FL
    10,
    11,
    12,  # FR
    15,
    16,
    17,  # BL
    19,
    20,
    21  # BR
]

maxVelocity = 100
mode = p.POSITION_CONTROL
p.setJointMotorControlArray(bodyUniqueId=boxId,
                            jointIndices=movingJoints,
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=np.zeros(12),
                            targetVelocities=np.zeros(12),
                            forces=np.ones(12) * np.inf)

counter = 0
angle1 = -np.pi / 2.0
angle2 = 0.0

angle = angle1

for i in range(100000000):
    counter += 1
    if counter % 1000 == 0:
        p.setJointMotorControlArray(
            bodyUniqueId=boxId,
            jointIndices=[8, 12, 17, 21],  # FWrists
            controlMode=p.POSITION_CONTROL,
            targetPositions=np.ones(4) * angle,
            targetVelocities=np.zeros(4),
            forces=np.ones(4) * 0.15)
        counter = 0
        if angle == angle1:
            angle = angle2
        else:
            angle = angle1
    p.stepSimulation()
p.disconnect()