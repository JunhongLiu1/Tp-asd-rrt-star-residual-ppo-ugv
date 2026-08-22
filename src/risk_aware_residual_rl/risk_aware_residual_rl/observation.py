"""ROS-independent residual-policy observation definition."""

from dataclasses import dataclass
import math
from typing import Tuple


OBSERVATION_FIELDS = (
    "baseline_linear",
    "actual_linear",
    "baseline_angular",
    "actual_angular",
    "linear_tracking_error",
    "angular_tracking_error",
    "lateral_error",
    "heading_error",
    "curvature",
    "goal_distance",
    "radiation_dose_rate",
    "terrain_impedance",
    "baseline_saturated",
    "safety_stop_active",
)


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class ObservationConfig:
    """Normalization scales for the fixed-order policy observation."""

    linear_speed_scale: float = 0.20
    angular_speed_scale: float = 0.60
    lateral_error_scale: float = 1.50
    heading_error_scale: float = 1.0
    curvature_scale: float = 3.0
    goal_distance_scale: float = 2.0
    radiation_scale: float = 0.5
    terrain_scale: float = 50.0

    def __post_init__(self):
        values = (
            self.linear_speed_scale,
            self.angular_speed_scale,
            self.lateral_error_scale,
            self.heading_error_scale,
            self.curvature_scale,
            self.goal_distance_scale,
            self.radiation_scale,
            self.terrain_scale,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("observation scales must be finite and positive")


@dataclass(frozen=True)
class Observation:
    """Unnormalized state consumed by the residual policy encoder."""

    baseline_linear: float
    actual_linear: float
    baseline_angular: float
    actual_angular: float
    lateral_error: float
    heading_error: float
    curvature: float
    goal_distance: float
    radiation_dose_rate: float
    terrain_impedance: float
    baseline_saturated: bool = False
    safety_stop_active: bool = False


@dataclass(frozen=True)
class ObservationEncoding:
    """Normalized observation and fail-closed inference validity."""

    vector: Tuple[float, ...]
    valid: bool
    fallback_reason: str = ""


def encode_observation(
    observation: Observation,
    config: ObservationConfig = ObservationConfig(),
) -> ObservationEncoding:
    """Encode and clip an observation into a stable fixed-order vector.

    Non-finite sensor/control values yield an all-zero invalid vector. The
    inference wrapper must then preserve the deterministic baseline command.
    """

    continuous_values = (
        observation.baseline_linear,
        observation.actual_linear,
        observation.baseline_angular,
        observation.actual_angular,
        observation.lateral_error,
        observation.heading_error,
        observation.curvature,
        observation.goal_distance,
        observation.radiation_dose_rate,
        observation.terrain_impedance,
    )
    if not all(math.isfinite(value) for value in continuous_values):
        return ObservationEncoding(
            vector=(0.0,) * len(OBSERVATION_FIELDS),
            valid=False,
            fallback_reason="nonfinite_observation",
        )

    linear_error = (
        observation.baseline_linear - observation.actual_linear
    )
    angular_error = (
        observation.baseline_angular - observation.actual_angular
    )
    raw_normalized = (
        observation.baseline_linear / config.linear_speed_scale,
        observation.actual_linear / config.linear_speed_scale,
        observation.baseline_angular / config.angular_speed_scale,
        observation.actual_angular / config.angular_speed_scale,
        linear_error / config.linear_speed_scale,
        angular_error / config.angular_speed_scale,
        observation.lateral_error / config.lateral_error_scale,
        observation.heading_error / config.heading_error_scale,
        observation.curvature / config.curvature_scale,
        observation.goal_distance / config.goal_distance_scale,
        observation.radiation_dose_rate / config.radiation_scale,
        observation.terrain_impedance / config.terrain_scale,
        1.0 if observation.baseline_saturated else 0.0,
        1.0 if observation.safety_stop_active else 0.0,
    )
    return ObservationEncoding(
        vector=tuple(_clamp(value, -1.0, 1.0) for value in raw_normalized),
        valid=True,
    )
