import numpy as np
import time
from spotmicro.Kinematics.LieAlgebra import TransToRp
import copy

STANCE = 0
SWING = 1


class BezierGait():
    def __init__(self):
        self.phi = 0.0
        self.lastTime = 0.0
        self.alpha = 0.0
        self.s = SWING
        self.NumBrezierPoints = 6

    def BrezierPoint(self, t, k, point):
        return point * self.Binomial(k) * np.power(t, k) * np.power(
            1 - t, self.NumBrezierPoints - k)

    def Binomial(self, k):
        return np.math.factorial(self.NumBrezierPoints) / (
            np.math.factorial(k) *
            np.math.factorial(self.NumBrezierPoints - k))

    def BrezierCoordinates(self, phi, L, LateralFraction, height=0.08):
        """ Calculate forward step coordinates.

            Inputs:
                phi: current trajectory phase
                L: step length
                LateralFraction: determines how 'lateral' the movement is
                height: maximum foot height during swing phase

            Outputs:
                X,Y,Z Foot coordinates
                Swing or Stance phase indicator
        """
        # Clip Phase
        if phi > 1.0:
            phi -= 1.0

        # Polar Leg Coords
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)

        if (phi >= 0.3 and phi < 0.8):
            self.s = STANCE

        elif (phi >= 0.8 or phi < 0.3):
            self.s = SWING

        # Brezier Curve Points (6 pts)
        # FORWARD
        X = np.array([
            0., -X_POLAR * L * 0.1, -X_POLAR * L * 0.05, X_POLAR * L * 0.15,
            X_POLAR * L * 0.05, X_POLAR * L * 0.1, 0.
        ])
        # LATERAL
        Y = np.array([
            0., -Y_POLAR * L * 0.1, -Y_POLAR * L * 0.05, Y_POLAR * L * 0.15,
            Y_POLAR * L * 0.05, Y_POLAR * L * 0.1, 0.
        ])
        # VERTICAL
        Z = np.array([
            0.,
            np.abs(L) * height * (2.0 / 3.0),
            np.abs(L) * height,
            np.abs(L) * height,
            np.abs(L) * height,
            np.abs(L) * height * (2.0 / 3.0),
            0.
        ])
        stepX = 0.
        stepY = 0.
        stepZ = 0.
        for i in range(self.NumBrezierPoints):
            stepX += self.BrezierPoint(phi, i, X[i])
            stepY += self.BrezierPoint(phi, i, Y[i])
            stepZ += self.BrezierPoint(phi, i, Z[i])

        return stepX, stepY, stepZ, self.s

    def GetFootStep(self, phi, L, LateralFraction, Lrot, T_bf):
        # Get Foot Coordinates for Forward Motion
        stepX_long, stepY_long, stepZ_long, s = self.BrezierCoordinates(
            phi, L, LateralFraction)

        r = np.sqrt(T_bf[0]**2 + T_bf[1]**2)

        # Rotation Circumferance
        RotationFraction = np.arctan2(T_bf[1], T_bf[0])

        stepX_rot, stepY_rot, stepZ_rot, s = self.BrezierCoordinates(
            phi, Lrot, np.pi / 2 + RotationFraction - self.alpha)

        # Sign for each quadrant
        if (T_bf[1] > 0):
            if (stepX_rot < 0):
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2),
                                         r)
            else:
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2),
                                        r)
        else:
            if (stepX_rot < 0):
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2),
                                        r)
            else:
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2),
                                         r)

        coord = np.array([
            stepX_long + stepX_rot, stepY_long + stepY_rot,
            stepZ_long + stepZ_rot
        ])

        return coord, s

    def GenerateTrajectory(self, L, LateralFraction, Lrot, T, offset, T_bf_):
        if (self.phi >= 0.99):
            self.lastTime = time.time()
        self.phi = (time.time() - self.lastTime) / T

        S = []
        T_bf = copy.deepcopy(T_bf_)
        for i, (key, Tbf_in) in enumerate(T_bf_.items()):
            _, p_bf = TransToRp(Tbf_in)
            step_coord, s = self.GetFootStep(self.phi + offset[i], L,
                                             LateralFraction, Lrot, p_bf)
            T_bf[key][0, 3] = Tbf_in[0, 3] + step_coord[0]
            T_bf[key][1, 3] = Tbf_in[1, 3] + step_coord[1]
            T_bf[key][2, 3] = Tbf_in[2, 3] + step_coord[2]
            S.append(s)
            # print("----------------------------------")
            # if s == STANCE:
            #     print("LEG {}, MODE: STANCE".format(key))
            # elif s == SWING:
            #     print("LEG {}, MODE: SWING".format(key))

            # print("EXTRA LEG COORDS: \t X: {}     Y: {}     Z: {}".format(
            #     step_coord[0], step_coord[1], step_coord[2]))

            # print("FINAL LEG COORDS: \n {}".format(T_bf[key]))
        return T_bf, S
