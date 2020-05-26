import numpy as np
import time
from spotmicro.Kinematics.LieAlgebra import TransToRp
import copy

STANCE = 0
SWING = 1


class BezierGait():
    def __init__(self, dSref=[0.0, 0.0, 0.5, 0.5], dt=0.01, Tstance=0.5):
        # Phase Lag Per Leg: FL, FR, BL, BR
        # Reference Leg is FL, always 0
        self.dSref = dSref
        self.alpha = 0.0
        # Number of control points is n + 1 = 11 + 1 = 12
        self.NumBezierPoints = 11
        # Timestep
        self.dt = dt

        # Total Elapsed Time
        self.time = 0.0
        # Touchdown Time
        self.TD_time = 0.0
        # Time Since Last Touchdown
        self.time_since_last_TD = 0.0
        # Trajectory Mode
        self.StanceSwing = SWING
        # Swing Phase value [0, 1] of Reference Foot
        self.SwRef = 0.0
        # Whether Reference Foot has Touched Down
        self.TD = False

        # Stance Time
        self.Tstance = Tstance

    def GetPhase(self, index, Tstance, Tswing):
        Sw_phase = 0.0
        Tstride = Tstance + Tswing
        ti = self.Get_ti(index, Tstride)
        # STANCE
        if ti >= 0.0 and ti <= Tstance:
            self.StanceSwing = STANCE
            return ti / Tstance
        # SWING
        elif ti >= -Tswing and ti < 0.0:
            self.StanceSwing = SWING
            Sw_phase = (ti + Tswing) / Tswing
        elif ti > Tstance and ti <= Tstride:
            self.StanceSwing = SWING
            Sw_phase = (ti - Tstance) / Tswing
        # Touchdown at End of Swing
        if Sw_phase >= 1.0:
            Sw_phase = 1.0
        if index == 0:
            self.SwRef = Sw_phase
            # REF Touchdown at End of Swing
            if self.SwRef >= 0.998:
                self.TD = True
                print("TOUCHDOWN")
        return Sw_phase

    def Get_ti(self, index, Tstride):
        return self.time_since_last_TD - self.dSref[index] * Tstride

    def Increment(self, dt, Tstride):
        """ dt: the time step
            Tstride: the total stride period (stance + swing)
        """
        self.CheckTouchDown()
        self.time_since_last_TD = self.time - self.TD_time
        if self.time_since_last_TD > Tstride:
            self.time_since_last_TD = Tstride
        print("T STRIDE: {}".format(Tstride))
        # Increment Time at the end in case TD just happened
        # So that we get time_since_last_TD = 0.0
        self.time += dt

        # If Tstride = Tstance, Tswing = 0
        # RESET ALL
        if Tstride == self.Tstance:
            self.time = 0.0
            self.time_since_last_TD = 0.0
            self.TD_time = 0.0
            self.SwRef = 0.0

    def CheckTouchDown(self):
        if self.SwRef >= 0.9 and self.TD:
            self.TD_time = self.time
            self.TD = False

    def BezierPoint(self, t, k, point):
        return point * self.Binomial(k) * np.power(t, k) * np.power(
            1 - t, self.NumBezierPoints - k)

    def Binomial(self, k):
        return np.math.factorial(self.NumBezierPoints) / (
            np.math.factorial(k) * np.math.factorial(self.NumBezierPoints - k))

    def BezierSwing(self, phase, L, LateralFraction, clearance_height=0.04):
        """ Calculate forward step coordinates.

            Inputs:
                phase: current trajectory phase
                L: step length
                LateralFraction: determines how 'lateral' the movement is
                clearance_height: maximum foot clearance_height during swing phase

            Outputs:
                X,Y,Z Foot coordinates
                Swing or Stance phase indicator
        """

        # Polar Leg Coords
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)

        # Bezier Curve Points (12 pts)
        # NOTE: L is HALF of STEP LENGTH
        # Forward Component
        X = np.array([
            -L,  # Ctrl Point 0, half of stride len
            -L * 1.4,  # Ctrl Point 1 diff btwn 1 and 0 = x Lift vel
            -L * 1.5,  # Ctrl Pts 2, 3, 4 are overlapped for
            -L * 1.5,  # Direction change after
            -L * 1.5,  # Follow Through
            0.0,  # Change acceleration during Protraction
            0.0,  # So we include three
            0.0,  # Overlapped Ctrl Pts: 5, 6, 7
            L * 1.5,  # Changing direction for swing-leg retraction
            L * 1.5,  # requires double overlapped Ctrl Pts: 8, 9
            L * 1.4,  # Swing Leg Retraction Velocity = Ctrl 11 - 10
            L
        ])
        # Account for lateral movements by multiplying with polar coord.
        # LateralFraction switches leg movements from X over to Y+ or Y-
        # As it tends away from zero
        X *= X_POLAR

        # Same as X but Lateral
        Y = np.array([
            -L,  # Ctrl Point 0, half of stride len
            -L * 1.4,  # Ctrl Point 1 diff btwn 1 and 0 = x Lift vel
            -L * 1.5,  # Ctrl Pts 2, 3, 4 are overlapped for
            -L * 1.5,  # Direction change after
            -L * 1.5,  # Follow Through
            0.0,  # Change acceleration during Protraction
            0.0,  # So we include three
            0.0,  # Overlapped Ctrl Pts: 5, 6, 7
            L * 1.5,  # Changing direction for swing-leg retraction
            L * 1.5,  # requires double overlapped Ctrl Pts: 8, 9
            L * 1.4,  # Swing Leg Retraction Velocity = Ctrl 11 - 10
            L
        ])
        # Account for lateral movements by multiplying with polar coord.
        # LateralFraction switches leg movements from X over to Y+ or Y-
        # As it tends away from zero
        Y *= Y_POLAR

        # Vertical Component
        Z = np.array([
            0.0,  # Double Overlapped Ctrl Pts for zero Lift
            0.0,  # Veloicty wrt hip (Pts 0 and 1)
            clearance_height * 0.9,  # Triple overlapped control for change in
            clearance_height * 0.9,  # Force direction during transition from
            clearance_height * 0.9,  # follow-through to protraction (2, 3, 4)
            clearance_height * 0.9,  # Double Overlapped Ctrl Pts for Traj
            clearance_height * 0.9,  # Dirctn Change during Protraction (5, 6)
            clearance_height * 1.1,  # Maximum Clearance at mid Traj, Pt 7
            clearance_height * 1.1,  # Smooth Transition from Protraction
            clearance_height * 1.1,  # To Retraction, Two Ctrl Pts (8, 9)
            0.0,  # Double Overlap Ctrl Pts for 0 Touchdown
            0.0,  # Veloicty wrt hip (Pts 10 and 11)
        ])

        stepX = 0.
        stepY = 0.
        stepZ = 0.
        for i in range(len(X)):
            stepX += self.BezierPoint(phase, i, X[i])
            stepY += self.BezierPoint(phase, i, Y[i])
            stepZ += self.BezierPoint(phase, i, Z[i])

        return stepX, stepY, stepZ

    def SineStance(self, phase, L, LateralFraction, penetration_depth=0.006):
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)
        # moves from +L to -L
        step = L * (1.0 - 2.0 * phase)
        stepX = step * X_POLAR
        stepY = step * Y_POLAR
        if L != 0.0:
            stepZ = -penetration_depth * np.cos(
                (np.pi * (stepX + stepY)) / (2.0 * L))
        else:
            stepZ = 0.0
        return stepX, stepY, stepZ

    def SwingStep(self, phase, L, LateralFraction, Lrot, clearance_height,
                  T_bf):
        # Get Foot Coordinates for Forward Motion
        stepX_long, stepY_long, stepZ_long = self.BezierSwing(
            phase, L, LateralFraction, clearance_height)

        r = np.sqrt(T_bf[0]**2 + T_bf[1]**2)

        # Rotation Circumferance
        RotationFraction = np.arctan2(T_bf[1], T_bf[0])

        stepX_rot, stepY_rot, stepZ_rot = self.BezierSwing(
            phase, Lrot, np.pi / 2 + RotationFraction - self.alpha,
            clearance_height)

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

        return coord

    def StanceStep(self, phase, L, LateralFraction, Lrot, penetration_depth,
                   T_bf):
        # Get Foot Coordinates for Forward Motion
        stepX_long, stepY_long, stepZ_long = self.SineStance(
            phase, L, LateralFraction, penetration_depth)

        r = np.sqrt(T_bf[0]**2 + T_bf[1]**2)

        # Rotation Circumferance
        RotationFraction = np.arctan2(T_bf[1], T_bf[0])

        stepX_rot, stepY_rot, stepZ_rot = self.SineStance(
            phase, Lrot, np.pi / 2 + RotationFraction - self.alpha,
            penetration_depth)

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

        return coord

    def GetFootStep(self, L, LateralFraction, Lrot, clearance_height,
                    penetration_depth, Tswing, T_bf, index):
        phase = self.GetPhase(index, self.Tstance, Tswing)
        if self.StanceSwing == STANCE:
            return self.StanceStep(phase, L, LateralFraction, Lrot,
                                   penetration_depth, T_bf)
        elif self.StanceSwing == SWING:
            return self.SwingStep(phase, L, LateralFraction, Lrot,
                                  clearance_height, T_bf)

    def GenerateTrajectory(self,
                           L,
                           LateralFraction,
                           Lrot,
                           vel,
                           T_bf_,
                           clearance_height=0.04,
                           penetration_depth=0.005,
                           dt=None):
        # First, get Tswing from desired speed and stride length
        # NOTE: L is HALF of stride length
        if vel != 0.0:
            Tswing = 2.0 * abs(L) / abs(vel)
        else:
            Tswing = 0.0
            L = 0.0

        # Then, get time since last Touchdown and increment time counter
        if dt is None:
            dt = self.dt

        # Catch infeasible timesteps
        if Tswing < dt:
            Tswing = 0.0
            L = 0.0

        self.Increment(dt, Tswing + self.Tstance)

        T_bf = copy.deepcopy(T_bf_)
        for i, (key, Tbf_in) in enumerate(T_bf_.items()):
            _, p_bf = TransToRp(Tbf_in)
            step_coord = self.GetFootStep(L, LateralFraction, Lrot,
                                          clearance_height, penetration_depth,
                                          Tswing, p_bf, i)
            if key == "FL":
                print("FL IS IN PASE: {}".format(self.StanceSwing))
                print("TIME: {}".format(self.time))
                print("TIME SINCE LAST TD: {}".format(self.time_since_last_TD))
                print("SWING REF: {}".format(self.SwRef))
                print("TSWING: {}".format(Tswing))
                print("-----------------------------------")
            T_bf[key][0, 3] = Tbf_in[0, 3] + step_coord[0]
            T_bf[key][1, 3] = Tbf_in[1, 3] + step_coord[1]
            T_bf[key][2, 3] = Tbf_in[2, 3] + step_coord[2]
        return T_bf
