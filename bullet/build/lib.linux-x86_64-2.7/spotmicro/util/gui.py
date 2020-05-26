#!/usr/bin/env python

import pybullet as pb
import time
import numpy as np
import sys


class GUI:
    def __init__(self, quadruped):

        time.sleep(0.5)

        self.cyaw = 90
        self.cpitch = -7
        self.cdist = 0.66

        self.xId = pb.addUserDebugParameter("x", -0.10, 0.10, 0.)
        self.yId = pb.addUserDebugParameter("y", -0.10, 0.10, 0.)
        self.zId = pb.addUserDebugParameter("z", -0.10, 0.10, 0.)
        self.rollId = pb.addUserDebugParameter("roll", -np.pi / 4, np.pi / 4,
                                               0.)
        self.pitchId = pb.addUserDebugParameter("pitch", -np.pi / 4, np.pi / 4,
                                                0.)
        self.yawId = pb.addUserDebugParameter("yaw", -np.pi / 4, np.pi / 4, 0.)
        self.StepLengthID = pb.addUserDebugParameter("Step Length", -0.5, 1.5,
                                                     0.)
        self.StepRotationId = pb.addUserDebugParameter("Step Rotation",
                                                       -np.pi / 2.0,
                                                       np.pi / 2.0, 0.)
        self.StepDirectionId = pb.addUserDebugParameter(
            "Step Direction", -np.pi / 2.0, np.pi / 2.0, 0.)
        self.StepPeriodId = pb.addUserDebugParameter("Step Period", 0.1, 3.,
                                                     2.5)

        self.quadruped = quadruped

    def UserInput(self):

        cubePos, cubeOrn = pb.getBasePositionAndOrientation(self.quadruped)
        pb.resetDebugVisualizerCamera(cameraDistance=self.cdist,
                                      cameraYaw=self.cyaw,
                                      cameraPitch=self.cpitch,
                                      cameraTargetPosition=cubePos)
        keys = pb.getKeyboardEvents()
        # Keys to change camera
        if keys.get(100):  # D
            self.cyaw += 1
        if keys.get(97):  # A
            self.cyaw -= 1
        if keys.get(99):  # C
            self.cpitch += 1
        if keys.get(102):  # F
            self.cpitch -= 1
        if keys.get(122):  # Z
            self.cdist += .01
        if keys.get(120):  # X
            self.cdist -= .01
        if keys.get(27):  # ESC
            pb.disconnect()
            sys.exit()

        # Read Robot Transform from GUI
        pos = np.array([
            pb.readUserDebugParameter(self.xId),
            pb.readUserDebugParameter(self.yId),
            pb.readUserDebugParameter(self.zId)
        ])
        orn = np.array([
            pb.readUserDebugParameter(self.rollId),
            pb.readUserDebugParameter(self.pitchId),
            pb.readUserDebugParameter(self.yawId)
        ])
        StepLength = pb.readUserDebugParameter(self.StepLengthID)
        StepRotation = pb.readUserDebugParameter(self.StepRotationId)
        StepDirection = pb.readUserDebugParameter(self.StepDirectionId)
        StepPeriod = pb.readUserDebugParameter(self.StepPeriodId)

        return pos, orn, StepLength, StepDirection, StepRotation, StepPeriod
