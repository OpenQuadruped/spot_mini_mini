#!/usr/bin/env python
# https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot
import numpy as np


class LegIK():
    def __init__(self,
                 legtype="RIGHT",
                 hip_length,
                 shoulder_length,
                 leg_length,
                 hip_lim,
                 shoulder_lim,
                 leg_lim):
        self.legtype = legtype
        self.hip_length = hip_length
        self.shoulder_length = shoulder_length
        self.leg_length = leg_length
        self.hip_lim = hip_lim
        self.shoulder_lim = shoulder_lim
        self.leg_lim = leg_lim

    def get_domain(xyz_coord):
        D = (y**2 + (-z)**2 - self.hip_length**2 +
             (-x)**2 - self.shoulder_length**2 -
             self.leg_length**2) / (2 * self.leg_length * self.shoulder_length)
        if D > 1 or D < -1:
            # DOMAIN BREACHED
            if D > 1:
                D = 0.99
                return D
            elif D < -1:
                D = -0.99
                return D
        else:
            return D

    def solve(self, xyz_coord):
        if self.legtype == "RIGHT":
            return RightIK(xyz_coord)
        else:
            return LeftIK(xyz_coord)

    def RightIK(self, xyz_coord):
        D = self.get_domain(xyz_coord)
        x = xyz_coord[0]
        y = xyz_coord[1]
        z = xyz_coord[2]
        leg_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        hip_angle = -np.arctan2(z, y) - np.arctan2(
            np.sqrt(y**2 + (-z)**2 - self.hip_length**2), -self.hip_length)
        shoulder_angle = np.arctan2(
            -x, np.sqrt(y**2 + (-z)**2 - self.hip_length**2)) - np.arctan2(
                self.leg_length * np.sin(leg_angle),
                self.shoulder_length + self.leg_length * np.cos(leg_angle))
        joint_angles = np.array([-hip_angle, shoulder_angle, leg_angle])
        return joint_angles

    def LeftIK(self, xyz_coord):
        D = self.get_domain(xyz_coord)
        x = xyz_coord[0]
        y = xyz_coord[1]
        z = xyz_coord[2]
        leg_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        hip_angle = -np.arctan2(z, y) - np.arctan2(
            np.sqrt(y**2 + (-z)**2 - self.hip_length**2), self.hip_length)
        shoulder_angle = np.arctan2(
            -x, np.sqrt(y**2 + (-z)**2 - self.hip_length**2)) - np.arctan2(
                self.leg_length * np.sin(leg_angle),
                self.shoulder_length + self.leg_length * np.cos(leg_angle))
        joint_angles = np.array([-hip_angle, shoulder_angle, leg_angle])
        return joint_angles
