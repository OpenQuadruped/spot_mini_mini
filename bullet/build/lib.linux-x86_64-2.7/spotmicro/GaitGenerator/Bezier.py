import numpy as np
import time

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

    def BrezierCoordinates(self, phi, L, angle, height=0.08):
        """ Calculate forward step coordinates.

            Inputs:
                phi: current trajectory phase
                L: step length
                angle: determines motion polarity [step direction]
                height: maximum foot height during swing phase

            Outputs:
                X,Y,Z Foot coordinates
                Swing or Stance phase indicator
        """
        # Clip Phase
        if phi > 1.0:
            phi -= 1.0

        # Polar Leg Coords
        X_POLAR = np.cos(angle)
        Y_POLAR = np.sin(angle)

        if (phi >= 0.3 and phi < 0.8):
            self.s = STANCE

        elif (phi >= 0.8 or phi < 0.3):
            self.s = SWING

        # Brezier Curve Points (6 pts)
        X = np.array([
            0., -X_POLAR * L * 0.1, -X_POLAR * L * 0.05, X_POLAR * L * 0.15,
            X_POLAR * L * 0.05, X_POLAR * L * 0.1, 0.
        ])
        Y = np.array([
            0., -Y_POLAR * L * 0.1, -Y_POLAR * L * 0.05, Y_POLAR * L * 0.15,
            Y_POLAR * L * 0.05, Y_POLAR * L * 0.1, 0.
        ])
        Z = np.array([0., 0., 0., np.abs(L) * height, 0., 0., 0.])
        stepX = 0.
        stepY = 0.
        stepZ = 0.
        for i in range(self.NumBrezierPoints):
            stepX += self.BrezierPoint(phi, i, X[i])
            stepY += self.BrezierPoint(phi, i, Y[i])
            stepZ += self.BrezierPoint(phi, i, Z[i])

        return stepX, stepY, stepZ, self.s

    def GetFootStep(self, phi, L, angle, Lrot, centerToFoot):

        # Get Foot Coordinates for Forward Motion
        stepX_long, stepY_long, stepZ_long, s = self.BrezierCoordinates(
            phi, L, angle)

        r = np.sqrt(centerToFoot[0]**2 + centerToFoot[1]**2)

        # Rotation Circumferance
        footAngle = np.arctan2(centerToFoot[1], centerToFoot[0])

        stepX_rot, stepY_rot, stepZ_rot, s = self.BrezierCoordinates(
            phi, Lrot, np.pi / 2 + footAngle - self.alpha)

        # Sign for each quadrant
        if (centerToFoot[1] > 0):
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

    def GenerateTrajectory(self, L, angle, Lrot, T, offset, T_bf_):
        if (self.phi >= 0.99):
            self.lastTime = time.time()
        self.phi = (time.time() - self.lastTime) / T

        S = []
        T_bf = T_bf_
        for i, (key, Tbf_in) in enumerate(T_bf_.items()):
            step_coord, s = self.GetFootStep(
                self.phi + offset[i], L, angle, Lrot, Tbf_in)
            T_bf[key][0] = Tbf_in[0] + step_coord[0]
            T_bf[key][1] = Tbf_in[1] + step_coord[1]
            T_bf[key][2] = Tbf_in[2] + step_coord[2]
            S.append(s)
        return T_bf, S
