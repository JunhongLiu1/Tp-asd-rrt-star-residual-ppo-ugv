"""Explicit reward and termination contract for residual control."""

from dataclasses import dataclass
from dataclasses import field
import math
from typing import Dict


@dataclass(frozen=True)
class RewardConfig:
    """Reward weights; safety penalties dominate nominal tracking rewards."""

    progress_weight: float = 8.0
    lateral_error_weight: float = 1.5
    heading_error_weight: float = 0.8
    speed_error_weight: float = 0.6
    residual_effort_weight: float = 0.10
    residual_change_weight: float = 0.20
    radiation_weight: float = 0.20
    terrain_weight: float = 0.05
    saturation_penalty: float = 0.5
    goal_bonus: float = 25.0
    safety_penalty: float = 100.0
    invalid_transition_penalty: float = 100.0

    def __post_init__(self):
        values = tuple(self.__dict__.values())
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("reward weights must be finite and non-negative")


@dataclass(frozen=True)
class TransitionMetrics:
    """One control transition used for reward and episode termination."""

    previous_goal_distance: float
    goal_distance: float
    lateral_error: float
    heading_error: float
    baseline_linear: float
    actual_linear: float
    linear_residual: float
    angular_residual: float
    previous_linear_residual: float = 0.0
    previous_angular_residual: float = 0.0
    radiation_dose_rate: float = 0.0
    terrain_impedance: float = 0.0
    saturated: bool = False
    goal_reached: bool = False
    safety_stop: bool = False
    collision: bool = False
    out_of_bounds: bool = False
    invalid_action: bool = False
    time_limit_reached: bool = False


@dataclass(frozen=True)
class RewardResult:
    """Reward value and Gymnasium-compatible episode status."""

    reward: float
    terminated: bool
    truncated: bool
    reason: str
    components: Dict[str, float] = field(default_factory=dict)


def _empty_components():
    return {
        "goal_bonus": 0.0,
        "heading_error": 0.0,
        "invalid_transition": 0.0,
        "lateral_error": 0.0,
        "progress": 0.0,
        "radiation": 0.0,
        "residual_change": 0.0,
        "residual_effort": 0.0,
        "safety": 0.0,
        "saturation": 0.0,
        "speed_error": 0.0,
        "terrain": 0.0,
    }


def _continuous_values(metrics):
    return (
        metrics.previous_goal_distance,
        metrics.goal_distance,
        metrics.lateral_error,
        metrics.heading_error,
        metrics.baseline_linear,
        metrics.actual_linear,
        metrics.linear_residual,
        metrics.angular_residual,
        metrics.previous_linear_residual,
        metrics.previous_angular_residual,
        metrics.radiation_dose_rate,
        metrics.terrain_impedance,
    )


def calculate_reward(
    metrics: TransitionMetrics,
    config: RewardConfig = RewardConfig(),
) -> RewardResult:
    """Calculate reward with terminal safety failures taking precedence."""

    if not all(math.isfinite(value) for value in _continuous_values(metrics)):
        components = _empty_components()
        components["invalid_transition"] = -abs(
            config.invalid_transition_penalty)
        return RewardResult(
            reward=sum(components.values()),
            terminated=True,
            truncated=False,
            reason="nonfinite_transition",
            components=components,
        )

    if (
        metrics.safety_stop or
        metrics.collision or
        metrics.out_of_bounds or
        metrics.invalid_action
    ):
        components = _empty_components()
        components["safety"] = -abs(config.safety_penalty)
        return RewardResult(
            reward=sum(components.values()),
            terminated=True,
            truncated=False,
            reason="safety_violation",
            components=components,
        )

    progress = metrics.previous_goal_distance - metrics.goal_distance
    speed_error = metrics.baseline_linear - metrics.actual_linear
    residual_effort = (
        metrics.linear_residual * metrics.linear_residual +
        metrics.angular_residual * metrics.angular_residual
    )
    residual_change = (
        (metrics.linear_residual - metrics.previous_linear_residual) ** 2 +
        (metrics.angular_residual - metrics.previous_angular_residual) ** 2
    )
    components = _empty_components()
    components["progress"] = config.progress_weight * progress
    components["lateral_error"] = (
        -config.lateral_error_weight * abs(metrics.lateral_error)
    )
    components["heading_error"] = (
        -config.heading_error_weight * abs(metrics.heading_error)
    )
    components["speed_error"] = (
        -config.speed_error_weight * abs(speed_error)
    )
    components["residual_effort"] = (
        -config.residual_effort_weight * residual_effort
    )
    components["residual_change"] = (
        -config.residual_change_weight * residual_change
    )
    components["radiation"] = (
        -config.radiation_weight * max(0.0, metrics.radiation_dose_rate)
    )
    components["terrain"] = (
        -config.terrain_weight *
        max(0.0, metrics.terrain_impedance) / 100.0
    )
    if metrics.saturated:
        components["saturation"] = -abs(config.saturation_penalty)
    if metrics.goal_reached:
        components["goal_bonus"] = abs(config.goal_bonus)
        return RewardResult(
            reward=sum(components.values()),
            terminated=True,
            truncated=False,
            reason="goal_reached",
            components=components,
        )
    reward = sum(components.values())
    if metrics.time_limit_reached:
        return RewardResult(
            reward=reward,
            terminated=False,
            truncated=True,
            reason="time_limit",
            components=components,
        )
    return RewardResult(
        reward=reward,
        terminated=False,
        truncated=False,
        reason="running",
        components=components,
    )
