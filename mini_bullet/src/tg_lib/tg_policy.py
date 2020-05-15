import numpy as np
from tg_lib.traj_gen import TrajectoryGenerator


class TGPolicy():
    """ state --> action
    """
    def __init__(
            self,
            movetype="walk",
            center_swing=0.0,
            amplitude_extension=0.2,
            amplitude_lift=0.4,
    ):
        """ movetype decides which type of
            TG we are training for

            OPTIONS:
                walk: one leg at a time LF, LB, RF, RB
                trot: LF|RB together followed by
                bound
                pace
                pronk
        """

        # Trajectory Generators
        self.TG_dict = {}

        movetype_dict = {
            "walk": [0, 0.25, 0.5, 0.75],  # LF | LB | RF | RB
            "trot": [0, 0.5, 0.5, 0],  # LF + RB | LB + RF
            "bound": [0, 0.5, 0, 0.5],  # LF + RF | LB + RB
            "pace": [0, 0, 0.5, 0.5],  # LF + LB | RF + RB
            "pronk": [0, 0, 0, 0]  # LF + LB + RF + RB
        }
        TG_LF = TrajectoryGenerator(center_swing, amplitude_extension,
                                    amplitude_lift, movetype_dict[movetype][0])
        TG_LB = TrajectoryGenerator(center_swing, amplitude_extension,
                                    amplitude_lift, movetype_dict[movetype][1])
        TG_RF = TrajectoryGenerator(center_swing, amplitude_extension,
                                    amplitude_lift, movetype_dict[movetype][2])
        TG_RB = TrajectoryGenerator(center_swing, amplitude_extension,
                                    amplitude_lift, movetype_dict[movetype][3])

        self.TG_dict["LF"] = TG_LF
        self.TG_dict["LB"] = TG_LB
        self.TG_dict["RF"] = TG_RF
        self.TG_dict["RB"] = TG_RB

    def increment(self, dt, f_tg, Beta):
        # Increment phase
        for (key, tg) in self.TG_dict.items():
            tg.CI.progress_tprime(dt, f_tg, Beta)

    def get_TG_state(self):
        # NOTE: MAYBE RETURN ONLY tprime for TG1 since that's
        # the 'master' phase
        # We get two observations per TG
        # obs = np.array([])
        # for i, (key, tg) in enumerate(self.TG_dict.items()):
        #     obs = np.append(obs, tg.get_state_based_on_phase[0])
        #     obs = np.append(obs, tg.get_state_based_on_phase[1])
        # return obs

        # OR just return phase, not sure why sin and cos is relevant...
        obs = np.array([])
        for i, (key, tg) in enumerate(self.TG_dict.items()):
            obs = np.append(obs, tg.CI.tprime)
        return obs

    def get_utg(self, action, alpha_tg, h_tg, intensity, num_motors):
        """ INPUTS:
                action: residuals for each motor
                        from Policy

                alpha_tg: swing amplitude from Policy
                h_tg: center extension from Policy
                      ie walking height

                num_motors: number of motors on minitaur

            OUTPUTS:
                action: residuals + TG
        """

        # Get Action from TG [no policies here]
        half_num_motors = int(num_motors / 2)
        for i, (key, tg) in enumerate(self.TG_dict.items()):
            action_idx = i
            swing, extend = tg.get_swing_extend_based_on_phase(
                alpha_tg, h_tg, intensity)
            # NOTE: ADDING to residuals
            action[action_idx] += swing
            action[action_idx + half_num_motors] += extend
        return action
