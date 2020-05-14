#!/usr/bin/env python

import numpy as np


class TrajectoryGenerator():
    def __init__(self):
        # Phase
        self.phi = 0.0

    def progress_phase(self, f_tg, dt):
        """ Eqn 1 in paper appendix

            dt: timestep

            POLICY PARAMS:
                f_tg frequency of TG
        """
        self.phi += (2 * np.pi * f_tg * dt) % (2 * np.pi)


class CyclicIntegrator():
    def __init__(self, TG, dphi_leg):
        self.TG = TG
        self.dphi_leg = dphi_leg
        self.phi_leg = TG.phi + self.dphi_leg % (2 * np.pi)

    def progress_phase(self, f_tg, dt):
        self.TG.progress_phase(f_tg, dt)
        self.phi_leg = self.TG.phi + self.dphi_leg % (2 * np.pi)
        self.tprime = 2.0 * np.pi - (2 * np.pi - self.phi_leg) / (
            2.0 * swing_stance_speed_ratio)

    def progress_tprime(self, dt, f_tg=1.0,
                        swing_stance_speed_ratio=1.0 / 3.0):
        """ swing_stance_speed_ratio is Beta in the paper, set by policy at each step, but default is 1/3
            delta_period is just dt

            This moves the phase based on delta (which is one parameter * delta_time_step).
            The speed of the phase depends on swing vs stance phase (phase > np.pi or phase < np.pi)  which has different speeds.
        """
        self.progress_phase(self, f_tg, dt)

        if self.phi_leg > 0.0 and self.phi_leg < 2.0 * np.pi * swing_stance_speed_ratio:
            self.tprime = self.phi_leg / (2.0 * (1 - swing_stance_speed_ratio))
        else:
            self.tprime = 2.0 * np.pi - (2 * np.pi - self.phi_leg) / (
                2.0 * swing_stance_speed_ratio)

    def get_state_base_on_phase(self):
        return [(np.cos(self.tprime) + 1) / 2.0,
                (np.sin(self.tprime) + 1) / 2.0]

    def get_swing_extend_based_on_phase(self,
                                        amplitude_swing,
                                        center_extension,
                                        theta=0.0):
        """ Eqn 2 in paper appendix

            Cs: center_swing
            Ae: amplitude_extension
            theta: extention difference between end of swing and stance (good for climbing)

            POLICY PARAMS:
                h_tg = center_extension
                alpha_th = amplitude_swing
        """
        amplitude_extension = self.amplitude_extension
        if self.tprime > np.pi:
            amplitude_extension = self.amplitude_lift

        # E(t)
        extend = center_extension + (amplitude_extension * np.sin(
            self.tprime)) * self.intensity + theta * np.cos(self.tprime)
        # S(t)
        swing = self.center_swing + amplitude_swing * np.cos(self.tprime)
        swing *= self.intensity
        return swing, extend
