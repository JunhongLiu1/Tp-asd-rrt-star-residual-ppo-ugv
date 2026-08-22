"""Fail-closed inference interfaces for residual policies."""

from dataclasses import dataclass
from pathlib import Path
from typing import Callable
from typing import Protocol
from typing import Sequence

from .action import ActionLimits
from .action import Command
from .action import ResidualApplication
from .action import apply_residual_action
from .errors import OptionalDependencyError
from .observation import Observation
from .observation import ObservationConfig
from .observation import encode_observation


class ResidualPolicy(Protocol):
    """Minimal inference contract implemented by learned and zero policies."""

    def predict(
        self, observation: Sequence[float], deterministic: bool = True
    ) -> Sequence[float]:
        """Return two normalized residual actions."""


class ZeroResidualPolicy:
    """Policy whose output is strictly equivalent to the PID baseline."""

    def predict(self, observation, deterministic=True):
        del observation, deterministic
        return (0.0, 0.0)


class CallableResidualPolicy:
    """Adapter for a test or deployment callable."""

    def __init__(self, function: Callable):
        self.function = function

    def predict(self, observation, deterministic=True):
        return self.function(observation, deterministic)


class StableBaselinesPolicy:
    """Optional Stable-Baselines3 model adapter with deferred imports."""

    def __init__(self, model):
        self.model = model

    @classmethod
    def load(cls, checkpoint, device="auto"):
        try:
            from stable_baselines3 import PPO
        except ImportError as exception:
            raise OptionalDependencyError(
                "PPO inference requires optional packages "
                "'stable_baselines3', 'torch', and 'gymnasium'."
            ) from exception
        return cls(PPO.load(checkpoint, device=device))

    def predict(self, observation, deterministic=True):
        action, unused_state = self.model.predict(
            observation, deterministic=deterministic)
        del unused_state
        if hasattr(action, "tolist"):
            action = action.tolist()
        if len(action) == 1 and hasattr(action[0], "__len__"):
            action = action[0]
        return action


def load_policy(policy_type, checkpoint_path=""):
    """Load only an explicit zero policy or an explicit SB3 checkpoint."""
    normalized_type = str(policy_type).strip().lower()
    checkpoint = str(checkpoint_path).strip()
    if normalized_type == "zero":
        return ZeroResidualPolicy()
    if normalized_type != "sb3":
        raise ValueError(
            "policy_type must be 'zero' or 'sb3'; random policies are not "
            "permitted")
    if not checkpoint:
        raise ValueError(
            "policy_type=sb3 requires an explicit checkpoint_path")
    path = Path(checkpoint).expanduser()
    if not path.is_file():
        raise ValueError(
            "residual policy checkpoint does not exist: " + str(path))
    return StableBaselinesPolicy.load(str(path))


@dataclass(frozen=True)
class PolicyDecision:
    """Inference decision and command selected by the safety envelope."""

    application: ResidualApplication
    observation_valid: bool
    policy_error: str = ""


class SafeResidualController:
    """Convert observations to bounded commands while preserving fallback."""

    def __init__(
        self,
        policy: ResidualPolicy,
        action_limits=ActionLimits(),
        observation_config=ObservationConfig(),
        deterministic=True,
    ):
        self.policy = policy
        self.action_limits = action_limits
        self.observation_config = observation_config
        self.deterministic = deterministic

    def command(self, baseline: Command, observation: Observation):
        """Return baseline after bad observation, inference, or action."""
        encoded = encode_observation(observation, self.observation_config)
        if not encoded.valid:
            return PolicyDecision(
                application=apply_residual_action(
                    baseline, (float("nan"), 0.0), self.action_limits),
                observation_valid=False,
                policy_error=encoded.fallback_reason,
            )
        try:
            action = self.policy.predict(
                encoded.vector, deterministic=self.deterministic)
        except Exception as exception:  # Deployment boundary must fail closed.
            return PolicyDecision(
                application=apply_residual_action(
                    baseline, (float("nan"), 0.0), self.action_limits),
                observation_valid=True,
                policy_error=(
                    "policy_exception:" + exception.__class__.__name__
                ),
            )
        return PolicyDecision(
            application=apply_residual_action(
                baseline, action, self.action_limits),
            observation_valid=True,
        )
