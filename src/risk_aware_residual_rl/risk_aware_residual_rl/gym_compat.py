"""Optional Gymnasium adapter for the ROS-independent core environment."""

from .core_env import ResidualControlCoreEnv
from .errors import OptionalDependencyError
from .observation import OBSERVATION_FIELDS


def _load_gym_dependencies():
    try:
        import gymnasium
        import numpy
    except ImportError as exception:
        raise OptionalDependencyError(
            "Gym interface requires optional packages 'gymnasium' and "
            "'numpy'. They are not downloaded automatically."
        ) from exception
    return gymnasium, numpy


def make_gym_env(core_env=None):
    """Return a Gymnasium Env wrapper or raise a clear dependency error."""
    gymnasium, numpy = _load_gym_dependencies()
    wrapped_core = core_env or ResidualControlCoreEnv()

    class ResidualControlGymEnv(gymnasium.Env):
        metadata = {"render_modes": []}

        def __init__(self):
            super().__init__()
            self.core_env = wrapped_core
            self.action_space = gymnasium.spaces.Box(
                low=-1.0,
                high=1.0,
                shape=(2,),
                dtype=numpy.float32,
            )
            self.observation_space = gymnasium.spaces.Box(
                low=-1.0,
                high=1.0,
                shape=(len(OBSERVATION_FIELDS),),
                dtype=numpy.float32,
            )

        def reset(self, *, seed=None, options=None):
            del options
            super().reset(seed=seed)
            observation, info = wrapped_core.reset(seed=seed)
            return numpy.asarray(observation, dtype=numpy.float32), info

        def step(self, action):
            observation, reward, terminated, truncated, info = (
                wrapped_core.step(action)
            )
            return (
                numpy.asarray(observation, dtype=numpy.float32),
                float(reward),
                bool(terminated),
                bool(truncated),
                info,
            )

        def close(self):
            return None

    return ResidualControlGymEnv()
