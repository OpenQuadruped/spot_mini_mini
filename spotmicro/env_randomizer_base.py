"""Abstract base class for environment randomizer.
Source: https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/minitaur/envs/env_randomizer_base.py
"""


import abc


class EnvRandomizerBase(object):
  """Abstract base class for environment randomizer.

  An EnvRandomizer is called in environment.reset(). It will
  randomize physical parameters of the objects in the simulation.
  The physical parameters will be fixed for that episode and be
  randomized again in the next environment.reset().
  """

  __metaclass__ = abc.ABCMeta

  @abc.abstractmethod
  def randomize_env(self, env):
    """Randomize the simulated_objects in the environment.

    Args:
      env: The environment to be randomized.
    """
    pass
