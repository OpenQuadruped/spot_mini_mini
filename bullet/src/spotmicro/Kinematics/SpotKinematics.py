#!/usr/bin/env python

import numpy as np
from LegKinematics import LegIK
from LieAlgebra import RpToTrans, TransToRp, TransInv, RPY, TransformVector
from collections import OrderedDict


class SpotModel:
    def __init__(self,
                 hip_length=0.04,
                 shoulder_length=0.1,
                 leg_length=0.1,
                 hip_lim=[-0.548, 0.548],
                 shoulder_lim=[-2.17, 0.97],
                 leg_lim=[-0.1, 2.59]):
        """ Spot Micro Kinematics
        """
        # Leg Parameters
        self.hip_length = hip_length
        self.shoulder_length = shoulder_length
        self.leg_length = leg_length

        # Leg Vector desired_positions

        # Distance Between Hips
        # Length
        self.hip_x = 0.192
        # Width
        self.hip_y = 0.075

        # Distance Between Feet
        # Length
        self.foot_x = 0.192
        # Width
        self.foot_y = 0.18

        # Body Height
        self.height = 0.15

        # Joint Parameters
        self.hip_lim = hip_lim
        self.shoulder_lim = shoulder_lim
        self.leg_lim = leg_lim

        # Dictionary to store Leg IK Solvers
        self.Legs = OrderedDict()
        self.Legs["FL"] = LegIK("LEFT", self.hip_length, self.shoulder_length,
                                self.leg_length, self.hip_lim,
                                self.shoulder_lim, self.leg_lim)
        self.Legs["FR"] = LegIK("RIGHT", self.hip_length, self.shoulder_length,
                                self.leg_length, self.hip_lim,
                                self.shoulder_lim, self.leg_lim)
        self.Legs["BL"] = LegIK("LEFT", self.hip_length, self.shoulder_length,
                                self.leg_length, self.hip_lim,
                                self.shoulder_lim, self.leg_lim)
        self.Legs["BR"] = LegIK("RIGHT", self.hip_length, self.shoulder_length,
                                self.leg_length, self.hip_lim,
                                self.shoulder_lim, self.leg_lim)

        # Dictionary to store Hip and Foot Transforms

        # Transform of Hip relative to world frame
        # With Body Centroid also in world frame
        Rwb = np.eye(3)
        self.WorldToHip = OrderedDict()

        self.ph_FL = np.array([self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["FL"] = RpToTrans(Rwb, self.ph_FL)

        self.ph_FR = np.array([self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["FR"] = RpToTrans(Rwb, self.ph_FR)

        self.ph_BL = np.array([-self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["BL"] = RpToTrans(Rwb, self.ph_BL)

        self.ph_BR = np.array([-self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["BR"] = RpToTrans(Rwb, self.ph_BR)

        # Transform of Foot relative to world frame
        # With Body Centroid also in world frame
        self.WorldToFoot = {}

        self.pf_FL = np.array(
            [self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FL"] = RpToTrans(Rwb, self.pf_FL)

        self.pf_FR = np.array(
            [self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FR"] = RpToTrans(Rwb, self.pf_FR)

        self.pf_BL = np.array(
            [-self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BL"] = RpToTrans(Rwb, self.pf_BL)

        self.pf_BR = np.array(
            [-self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BR"] = RpToTrans(Rwb, self.pf_BR)

    def IK(self, orn, pos, T_bf):
        """ Converts a desired position and orientation wrt Spot's
            home position, with a desired body-to-foot Transform
            into a body-to-hip Transform, of which the translational
            component can be fed into the LegIK solver.

            Finally, the resultant joint angles are returned
            from the LegIK solver for each leg.

        :param orn: A 3x1 np.array([]) with Spot's Roll, Pitch, Yaw angles
        :param pos: A 3x1 np.array([]) with Spot's X, Y, Z coordinates
        :param T_bf: Dictionary of desired body-to-foot Transforms.
        :return: Joint angles for each of Spot's joints.
        """

        # Following steps in attached document: SpotBodyIK.
        # TODO: LINK DOC

        # 4 legs, 3 joints per leg
        joint_angles = np.zeros((4, 3))

        # Only get Rot component
        Rb, _ = TransToRp(RPY(orn[0], orn[1], orn[2]))
        # print("Rb: ", Rb)
        pb = pos
        # print("pb:", pb)
        T_wb = RpToTrans(Rb, pb)
        # print("T_wb: ", T_wb)

        for i, (key, T_wh) in enumerate(self.WorldToHip.items()):
            # ORDER: FL, FR, FR, BL, BR

            # Extract vector component
            _, p_bf = TransToRp(T_bf[key])

            # Step 1, get T_bh for each leg
            T_bh = np.dot(TransInv(T_wb), T_wh)

            # Step 2, get T_hf for each leg

            # VECTOR ADDITION METHOD
            _, p_bh = TransToRp(T_bh)
            p_hf0 = p_bf - p_bh

            # TRANSFORM METHOD - UNCOMMENT TO USE
            T_hf = np.dot(TransInv(T_bh), T_bf[key])
            _, p_hf1 = TransToRp(T_hf)

            if p_hf1.all() != p_hf0.all():
                print("NOT EQUAL")

            p_hf = p_hf1

            # Step 3, compute joint angles from T_hf for each leg
            joint_angles[i, :] = self.Legs[key].solve(p_hf)

        return joint_angles
