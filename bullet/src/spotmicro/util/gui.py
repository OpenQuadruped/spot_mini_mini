#!/usr/bin/env python

import pybullet as pb
import time
import numpy as np


class GUI:
    def __init__(self):

        time.sleep(0.5)

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
        self.StepRotationId = pb.addUserDebugParameter("Step Rotation", -1.5,
                                                       1.5, 0.)
        self.StepDirectionId = pb.addUserDebugParameter(
            "Step Direction", -180., 180., 0.)
        self.StepPeriodId = pb.addUserDebugParameter("Step Period", 0.1, 3.,
                                                     2.5)

    def UserInput(self):

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
