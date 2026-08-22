"""Small deterministic, ROS-free environment for contract and PPO smoke tests.

This first-order surrogate is not a claim of Gazebo or real-vehicle fidelity.
Its purpose is to test observation/action/reward plumbing before a ROS adapter
is connected to the frozen Pure Pursuit/PID baseline.
"""

from dataclasses import dataclass
import math
import random

from .action import ActionLimits
from .action import Command
from .action import apply_residual_action
from .observation import Observation
from .observation import ObservationConfig
from .observation import encode_observation
from .reward import RewardConfig
from .reward import TransitionMetrics
from .reward import calculate_reward


@dataclass(frozen=True)
class CoreEnvConfig:
    """Configuration for the deterministic residual-control surrogate."""

    dt: float = 0.10
    max_steps: int = 400
    goal_tolerance: float = 0.20
    velocity_time_constant: float = 0.40
    nominal_linear_speed: float = 0.08
    minimum_tracking_speed: float = 0.04
    max_initial_goal_distance: float = 1.80
    min_initial_goal_distance: float = 0.80

    def __post_init__(self):
        values = (
            self.dt,
            self.goal_tolerance,
            self.velocity_time_constant,
            self.nominal_linear_speed,
            self.minimum_tracking_speed,
            self.max_initial_goal_distance,
            self.min_initial_goal_distance,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("environment configuration must be finite")
        if (
            self.dt <= 0.0 or
            self.max_steps <= 0 or
            self.goal_tolerance <= 0.0 or
            self.velocity_time_constant <= 0.0 or
            self.nominal_linear_speed < 0.0 or
            self.minimum_tracking_speed <= 0.0 or
            self.minimum_tracking_speed > self.nominal_linear_speed or
            self.min_initial_goal_distance <= self.goal_tolerance or
            self.max_initial_goal_distance < self.min_initial_goal_distance
        ):
            raise ValueError("environment configuration is invalid")


class ResidualControlCoreEnv:
    """Gym-shaped environment with no Gym, Torch, SB3, or ROS dependency."""

    def __init__(
        self,
        config=CoreEnvConfig(),
        action_limits=ActionLimits(),
        observation_config=ObservationConfig(),
        reward_config=RewardConfig(),
    ):
        self.config = config
        self.action_limits = action_limits
        self.observation_config = observation_config
        self.reward_config = reward_config
        self._rng = random.Random()
        self._seed = None
        self._reset_state()

    def _reset_state(self):
        self.steps = 0
        self.goal_distance = 0.0
        self.actual_linear = 0.0
        self.actual_angular = 0.0
        self.heading_error = 0.0
        self.lateral_error = 0.0
        self.curvature = 0.0
        self.radiation_dose_rate = 0.0
        self.terrain_impedance = 0.0
        self.previous_linear_residual = 0.0
        self.previous_angular_residual = 0.0
        self.last_baseline = Command(0.0, 0.0)
        self.done = False

    def _baseline_command(self):
        slowdown = min(
            1.0,
            max(
                0.0,
                (self.goal_distance - self.config.goal_tolerance) / 1.5,
            ),
        )
        terrain_factor = max(0.30, 1.0 - self.terrain_impedance / 140.0)
        linear = self.config.nominal_linear_speed * slowdown * terrain_factor
        if self.goal_distance > self.config.goal_tolerance:
            linear = max(self.config.minimum_tracking_speed, linear)
        angular = max(-0.50, min(0.50, self.curvature * linear))
        return Command(linear, angular)

    def _observation(self, baseline):
        return Observation(
            baseline_linear=baseline.linear,
            actual_linear=self.actual_linear,
            baseline_angular=baseline.angular,
            actual_angular=self.actual_angular,
            lateral_error=self.lateral_error,
            heading_error=self.heading_error,
            curvature=self.curvature,
            goal_distance=self.goal_distance,
            radiation_dose_rate=self.radiation_dose_rate,
            terrain_impedance=self.terrain_impedance,
            baseline_saturated=False,
            safety_stop_active=False,
        )

    def reset(self, seed=None):
        """Reset and return ``(observation, info)`` like Gymnasium."""
        if seed is not None:
            self._seed = int(seed)
            self._rng.seed(self._seed)
        elif self._seed is None:
            self._seed = 0
            self._rng.seed(self._seed)
        self._reset_state()
        self.goal_distance = self._rng.uniform(
            self.config.min_initial_goal_distance,
            self.config.max_initial_goal_distance,
        )
        self.heading_error = self._rng.uniform(-0.30, 0.30)
        self.lateral_error = self._rng.uniform(-0.20, 0.20)
        self.curvature = self._rng.uniform(-1.2, 1.2)
        self.radiation_dose_rate = self._rng.uniform(0.05, 0.40)
        self.terrain_impedance = self._rng.uniform(0.0, 45.0)
        self.last_baseline = self._baseline_command()
        encoded = encode_observation(
            self._observation(self.last_baseline), self.observation_config)
        return encoded.vector, {
            "seed": self._seed,
            "baseline": self.last_baseline,
        }

    def step(self, action):
        """Advance the surrogate by one bounded residual-control interval."""
        if self.done:
            raise RuntimeError("step() called after episode completion")

        baseline = self._baseline_command()
        applied = apply_residual_action(
            baseline, action, self.action_limits, enabled=True)
        previous_goal_distance = self.goal_distance
        alpha = min(1.0, self.config.dt / self.config.velocity_time_constant)
        self.actual_linear += alpha * (
            applied.command.linear - self.actual_linear)
        self.actual_angular += alpha * (
            applied.command.angular - self.actual_angular)

        forward_progress = max(
            0.0,
            self.actual_linear * math.cos(self.heading_error) * self.config.dt,
        )
        self.goal_distance = max(0.0, self.goal_distance - forward_progress)
        self.heading_error += (
            self.actual_angular - baseline.angular) * self.config.dt
        self.heading_error = max(-math.pi, min(math.pi, self.heading_error))
        self.lateral_error += (
            self.actual_linear * math.sin(self.heading_error) * self.config.dt
        )
        self.steps += 1

        goal_reached = self.goal_distance <= self.config.goal_tolerance
        time_limit_reached = self.steps >= self.config.max_steps
        metrics = TransitionMetrics(
            previous_goal_distance=previous_goal_distance,
            goal_distance=self.goal_distance,
            lateral_error=self.lateral_error,
            heading_error=self.heading_error,
            baseline_linear=baseline.linear,
            actual_linear=self.actual_linear,
            linear_residual=applied.residual.linear,
            angular_residual=applied.residual.angular,
            previous_linear_residual=self.previous_linear_residual,
            previous_angular_residual=self.previous_angular_residual,
            radiation_dose_rate=self.radiation_dose_rate,
            terrain_impedance=self.terrain_impedance,
            saturated=applied.clipped,
            goal_reached=goal_reached,
            invalid_action=not applied.valid_action,
            time_limit_reached=time_limit_reached,
        )
        reward = calculate_reward(metrics, self.reward_config)
        self.previous_linear_residual = applied.residual.linear
        self.previous_angular_residual = applied.residual.angular
        self.last_baseline = baseline
        self.done = reward.terminated or reward.truncated

        next_baseline = self._baseline_command()
        encoded = encode_observation(
            self._observation(next_baseline), self.observation_config)
        info = {
            "reason": reward.reason,
            "baseline": baseline,
            "command": applied.command,
            "residual": applied.residual,
            "valid_action": applied.valid_action,
            "fallback_reason": applied.fallback_reason,
            "action_clipped": applied.clipped,
            "reward_components": reward.components,
        }
        return (
            encoded.vector,
            reward.reward,
            reward.terminated,
            reward.truncated,
            info,
        )
