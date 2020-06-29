#!/usr/bin/env python
# https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot
import numpy as np


class LegIK():
    def __init__(self,
                 legtype="RIGHT",
                 hip_length=0.04,
                 shoulder_length=0.1,
                 leg_length=0.1,
                 hip_lim=[-0.548, 0.548],
                 shoulder_lim=[-2.17, 0.97],
                 leg_lim=[-0.1, 2.59]):
        self.legtype = legtype
        self.hip_length = hip_length
        self.shoulder_length = shoulder_length
        self.leg_length = leg_length
        self.hip_lim = hip_lim
        self.shoulder_lim = shoulder_lim
        self.leg_lim = leg_lim

    def get_domain(self, x, y, z):
        """
        Calculates the leg's Domain and caps it in case of a breach

        :param x,y,z: hip-to-foot distances in each dimension
        :return: Leg Domain D
        """
        D = (y**2 + (-z)**2 - self.hip_length**2 +
             (-x)**2 - self.shoulder_length**2 -
             self.leg_length**2) / (2 * self.leg_length * self.shoulder_length)
        if D > 1 or D < -1:
            # DOMAIN BREACHED
            print("---------DOMAIN BREACH---------")
            D = np.clip(D, -0.99, 0.99)
            return D
        else:
            return D

    def solve(self, xyz_coord):
        """
        Generic Leg Inverse Kinematics Solver

        :param xyz_coord: hip-to-foot distances in each dimension
        :return: Joint Angles required for desired position
        """
        x = xyz_coord[0]
        y = xyz_coord[1]
        z = xyz_coord[2]
        D = self.get_domain(x, y, z)
        if self.legtype == "RIGHT":
            return self.RightIK(x, y, z, D)
        else:
            return self.LeftIK(x, y, z, D)

    def RightIK(self, x, y, z, D):
        """
        Right Leg Inverse Kinematics Solver

        :param x,y,z: hip-to-foot distances in each dimension
        :param D: leg domain
        :return: Joint Angles required for desired position
        """
        leg_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        sqrt_component = y**2 + (-z)**2 - self.hip_length**2
        if sqrt_component < 0.0:
            print("NEGATIVE SQRT")
            sqrt_component = 0.0
        hip_angle = -np.arctan2(z, y) - np.arctan2(np.sqrt(sqrt_component),
                                                   -self.hip_length)
        shoulder_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.leg_length * np.sin(leg_angle),
            self.shoulder_length + self.leg_length * np.cos(leg_angle))
        joint_angles = np.array([hip_angle, -shoulder_angle, -leg_angle])
        return joint_angles

    def LeftIK(self, x, y, z, D):
        """
        Left Leg Inverse Kinematics Solver

        :param x,y,z: hip-to-foot distances in each dimension
        :param D: leg domain
        :return: Joint Angles required for desired position
        """
        leg_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        sqrt_component = y**2 + (-z)**2 - self.hip_length**2
        if sqrt_component < 0.0:
            print("NEGATIVE SQRT")
            sqrt_component = 0.0
        hip_angle = -np.arctan2(z, y) - np.arctan2(np.sqrt(sqrt_component),
                                                   self.hip_length)
        shoulder_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.leg_length * np.sin(leg_angle),
            self.shoulder_length + self.leg_length * np.cos(leg_angle))
        joint_angles = np.array([hip_angle, -shoulder_angle, -leg_angle])
        return joint_angles
