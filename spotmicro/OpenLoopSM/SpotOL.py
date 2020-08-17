""" Open Loop Controller for Spot Micro. Takes GUI params or uses default
"""
import numpy as np
from random import shuffle
import copy
# Ensuring totally random seed every step!
np.random.seed()

FB = 0
LAT = 1
ROT = 2
COMBI = 3

FWD = 0
ALL = 1


class BezierStepper():
    def __init__(self,
                 pos=np.array([0.0, 0.0, 0.0]),
                 orn=np.array([0.0, 0.0, 0.0]),
                 StepLength=0.04,
                 LateralFraction=0.0,
                 YawRate=0.0,
                 StepVelocity=0.001,
                 ClearanceHeight=0.045,
                 PenetrationDepth=0.003,
                 episode_length=5000,
                 dt=0.01,
                 num_shuffles=2,
                 mode=FWD):
        self.pos = pos
        self.orn = orn
        self.desired_StepLength = StepLength
        self.StepLength = StepLength
        self.StepLength_LIMITS = [-0.05, 0.05]
        self.LateralFraction = LateralFraction
        self.LateralFraction_LIMITS = [-np.pi / 2.0, np.pi / 2.0]
        self.YawRate = YawRate
        self.YawRate_LIMITS = [-1.0, 1.0]
        self.StepVelocity = StepVelocity
        self.StepVelocity_LIMITS = [0.1, 1.5]
        self.ClearanceHeight = ClearanceHeight
        self.ClearanceHeight_LIMITS = [0.0, 0.04]
        self.PenetrationDepth = PenetrationDepth
        self.PenetrationDepth_LIMITS = [0.0, 0.02]

        self.mode = mode

        self.dt = dt

        # Keep track of state machine
        self.time = 0
        # Decide how long to stay in each phase based on maxtime
        self.max_time = episode_length
        """ States
                1: FWD/BWD
                2: Lat
                3: Rot
                4: Combined
        """
        self.order = [FB, LAT, ROT, COMBI]
        # Shuffles list in place so the order of states is unpredictable
        # NOTE: increment num_shuffles by episode num (cap at 10
        # and reset or someting) for some forced randomness
        for _ in range(num_shuffles):
            shuffle(self.order)

        # Forward/Backward always needs to be first!
        self.reshuffle()

        # Current State
        self.current_state = self.order[0]

        # Divide by number of states (see RL_SM())
        self.time_per_episode = int(self.max_time / len(self.order))

    def ramp_up(self):
        if self.StepLength < self.desired_StepLength:
            self.StepLength += self.desired_StepLength * self.dt

    def reshuffle(self):
        self.time = 0
        # Make sure FWD/BWD is always first state
        FB_index = self.order.index(FB)
        if FB_index != 0:
            what_was_in_zero = self.order[0]
            self.order[0] = FB
            self.order[FB_index] = what_was_in_zero

    def which_state(self):
        # Ensuring totally random seed every step!
        np.random.seed()
        if self.time > self.max_time:
            # Combined
            self.current_state = COMBI
            self.time = 0
        else:
            index = int(self.time / self.time_per_episode)

            if index > len(self.order) - 1:
                index = len(self.order) - 1

            self.current_state = self.order[index]

    def StateMachine(self):
        """
        State Machined used for training robust RL on top of OL gait.

        STATES:
            Forward/Backward: All Default Values.
                              Can have slow changes to
                              StepLength(+-) and Velocity

            Lateral: As above (fwd or bwd random) with added random
                     slow changing LateralFraction param

            Rotating: As above except with YawRate

            Combined: ALL changeable values may change!
                            StepLength
                            StepVelocity
                            LateralFraction
                            YawRate

        NOTE: the RL is solely responsible for modulating Clearance Height
              and Penetration Depth
        """
        if self.mode is ALL:
            self.which_state()

            if self.current_state == FB:
                # print("FORWARD/BACKWARD")
                self.FB()
            elif self.current_state == LAT:
                # print("LATERAL")
                self.LAT()
            elif self.current_state == ROT:
                # print("ROTATION")
                self.ROT()
            elif self.current_state == COMBI:
                # print("COMBINED")
                self.COMBI()

        return self.return_bezier_params()

    def return_bezier_params(self):
        # First, Clip Everything
        self.StepLength = np.clip(self.StepLength, self.StepLength_LIMITS[0],
                                  self.StepLength_LIMITS[1])
        self.StepVelocity = np.clip(self.StepVelocity,
                                    self.StepVelocity_LIMITS[0],
                                    self.StepVelocity_LIMITS[1])
        self.LateralFraction = np.clip(self.LateralFraction,
                                       self.LateralFraction_LIMITS[0],
                                       self.LateralFraction_LIMITS[1])
        self.YawRate = np.clip(self.YawRate, self.YawRate_LIMITS[0],
                               self.YawRate_LIMITS[1])
        self.ClearanceHeight = np.clip(self.ClearanceHeight,
                                       self.ClearanceHeight_LIMITS[0],
                                       self.ClearanceHeight_LIMITS[1])
        self.PenetrationDepth = np.clip(self.PenetrationDepth,
                                        self.PenetrationDepth_LIMITS[0],
                                        self.PenetrationDepth_LIMITS[1])
        # Then, return
        # FIRST COPY TO AVOID OVERWRITING
        pos = copy.deepcopy(self.pos)
        orn = copy.deepcopy(self.orn)
        StepLength = copy.deepcopy(self.StepLength)
        LateralFraction = copy.deepcopy(self.LateralFraction)
        YawRate = copy.deepcopy(self.YawRate)
        StepVelocity = copy.deepcopy(self.StepVelocity)
        ClearanceHeight = copy.deepcopy(self.ClearanceHeight)
        PenetrationDepth = copy.deepcopy(self.PenetrationDepth)

        return pos, orn, StepLength, LateralFraction,\
            YawRate, StepVelocity,\
            ClearanceHeight, PenetrationDepth

    def FB(self):
        """
        Here, we can modulate StepLength and StepVelocity
        """
        # The maximum update amount for these element
        StepLength_DELTA = self.dt * (self.StepLength_LIMITS[1] -
                                      self.StepLength_LIMITS[0]) / (6.0)
        StepVelocity_DELTA = self.dt * (self.StepVelocity_LIMITS[1] -
                                        self.StepVelocity_LIMITS[0]) / (2.0)

        # Add either positive or negative or zero delta for each
        # NOTE: 'High' is open bracket ) so the max is 1
        if self.StepLength < -self.StepLength_LIMITS[0] / 2.0:
            StepLength_DIRECTION = np.random.randint(-1, 3, 1)[0]
        elif self.StepLength > self.StepLength_LIMITS[1] / 2.0:
            StepLength_DIRECTION = np.random.randint(-2, 2, 1)[0]
        else:
            StepLength_DIRECTION = np.random.randint(-1, 2, 1)[0]
        StepVelocity_DIRECTION = np.random.randint(-1, 2, 1)[0]

        # Now, modify modifiable params AND CLIP
        self.StepLength += StepLength_DIRECTION * StepLength_DELTA
        self.StepLength = np.clip(self.StepLength, self.StepLength_LIMITS[0],
                                  self.StepLength_LIMITS[1])
        self.StepVelocity += StepVelocity_DIRECTION * StepVelocity_DELTA
        self.StepVelocity = np.clip(self.StepVelocity,
                                    self.StepVelocity_LIMITS[0],
                                    self.StepVelocity_LIMITS[1])

    def LAT(self):
        """
        Here, we can modulate StepLength and LateralFraction
        """
        # The maximum update amount for these element
        LateralFraction_DELTA = self.dt * (self.LateralFraction_LIMITS[1] -
                                           self.LateralFraction_LIMITS[0]) / (
                                               2.0)

        # Add either positive or negative or zero delta for each
        # NOTE: 'High' is open bracket ) so the max is 1
        LateralFraction_DIRECTION = np.random.randint(-1, 2, 1)[0]

        # Now, modify modifiable params AND CLIP
        self.LateralFraction += LateralFraction_DIRECTION * LateralFraction_DELTA
        self.LateralFraction = np.clip(self.LateralFraction,
                                       self.LateralFraction_LIMITS[0],
                                       self.LateralFraction_LIMITS[1])

    def ROT(self):
        """
        Here, we can modulate StepLength and YawRate
        """
        # The maximum update amount for these element
        # no dt since YawRate is already mult by dt
        YawRate_DELTA = (self.YawRate_LIMITS[1] -
                         self.YawRate_LIMITS[0]) / (2.0)

        # Add either positive or negative or zero delta for each
        # NOTE: 'High' is open bracket ) so the max is 1
        YawRate_DIRECTION = np.random.randint(-1, 2, 1)[0]

        # Now, modify modifiable params AND CLIP
        self.YawRate += YawRate_DIRECTION * YawRate_DELTA
        self.YawRate = np.clip(self.YawRate, self.YawRate_LIMITS[0],
                               self.YawRate_LIMITS[1])

    def COMBI(self):
        """
        Here, we can modify all the parameters
        """

        self.FB()
        self.LAT()
        self.ROT()
