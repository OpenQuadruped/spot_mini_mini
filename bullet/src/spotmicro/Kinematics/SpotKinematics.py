#!/usr/bin/env python

import numpy as np
from spotmicro.Kinematics import LegIK
from spotmicro.LieAlgebra import RpToTrans, TransToRp, TransInv


class SpotKinematics:
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
        self.foot_y = 0.11

        # Body Height
        self.height = 0.15

        # Joint Parameters
        self.hip_lim = hip_lim
        self.shoulder_lim = shoulder_lim
        self.leg_lim = leg_lim

        # Dictionary to store Leg IK Solvers
        self.Legs = {}
        self.Legs["FL"] = LegIK("LEFT", self.hip_length, self.shoulder_length,
                                self.leg_length, self.hip_lim,
                                self.shoulder_lim, self.leg_lim)
        self.Legs["BL"] = LegIK("LEFT", self.hip_length, self.shoulder_length,
                                self.leg_length, self.hip_lim,
                                self.shoulder_lim, self.leg_lim)
        self.Legs["FR"] = LegIK("RIGHT", self.hip_length, self.shoulder_length,
                                self.leg_length, self.hip_lim,
                                self.shoulder_lim, self.leg_lim)
        self.Legs["BR"] = LegIK("RIGHT", self.hip_length, self.shoulder_length,
                                self.leg_length, self.hip_lim,
                                self.shoulder_lim, self.leg_lim)

        # Dictionary to store Hip and Foot Transforms

        # Transform of Hip relative to world frame
        # With Body Centroid also in world frame
        Rwb = np.eye(3)
        self.WorldToHip = {}

        ph_FL = np.array(
            [self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["FL"] = RpToTrans(Rwb, ph_FL)

        ph_BL = np.array(
            [-self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["BL"] = RpToTrans(Rwb, ph_BL)

        ph_FR = np.array(
            [self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["FR"] = RpToTrans(Rwb, ph_FR)

        ph_BR = np.array(
            [-self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["BR"] = RpToTrans(Rwb, ph_BR)

        # Transform of Foot relative to world frame
        # With Body Centroid also in world frame
        self.WorldToFoot = {}

        pf_FL = np.array(
            [self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FL"] = RpToTrans(Rwb, pf_FL)

        pf_BL = np.array(
            [-self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BL"] = RpToTrans(Rwb, pf_BL)

        pf_FR = np.array(
            [self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FR"] = RpToTrans(Rwb, pf_FR)

        pf_BR = np.array(
            [-self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BR"] = RpToTrans(Rwb, pf_BR)

    def IK(self, orn, pos, T_bf):
        """ Converts a desired position and orientation wrt Spot's
            home position, with a desired body-to-foot Transform
            into a body-to-hip Transform, of which the translational
            component can be fed into the LegIK solver.

            Finally, the resultant joint angles are returned
            from the LegIK solver for each leg.

        :param orn: A 3x1 np.array([]) with Spot's Roll, Pitch, Yaw angles
        :param pos: A 3x1 np.array([]) with Spot's X, Y, Z coordinates
        :param T_bf: The desired body-to-foot Transform.
        :return: Joint angles for each of Spot's joints.
        """

        # Following steps in attached document: SpotBodyIK.
        # TODO: LINK DOC

        # 4 legs, 3 joints per leg
        joint_angles = np.zeros((4, 3))

        Rb = np.diag(orn)
        pb = pos
        T_wb = RpToTrans(Rb, pb)

        # Extract vector component
        _, p_bf = TransToRp(T_bf)

        for i, (key, T_wh) in enumerate(self.WorldToHip.items()):
            # ORDER: FL, BL, FR, BR

            # Step 1, get T_bh for each leg
            T_bh = np.matmul(TransInv(T_wb), T_wh)

            # Step 2, get T_hf for each leg

            # VECTOR ADDITION METHOD - UNCOMMENT TO USE
            _, p_bh = TransToRp(T_bh)
            p_hf = p_bh + p_bf

            # TRANSFORM METHOD - UNCOMMENT TO USE
            T_hf = np.matmul(TransInv(T_bh), T_bf)
            _, p_hf = TransToRp(T_hf)

            # Step 3, compute joint angles from T_hf for each leg
            joint_angles[i, :] = self.Legs[key].solve(p_hf)

        return joint_angles
