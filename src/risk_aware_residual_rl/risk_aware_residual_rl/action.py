"""Safety envelope for applying a learned residual to a baseline command."""

from dataclasses import dataclass
import math
from typing import Iterable
from typing import Tuple


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class Command:
    """Planar velocity command in SI units."""

    linear: float
    angular: float

    @property
    def finite(self):
        """Return whether both command components are finite."""
        return math.isfinite(self.linear) and math.isfinite(self.angular)


@dataclass(frozen=True)
class ActionLimits:
    """Residual and absolute command bounds.

    Policy actions are normalized to ``[-1, 1]``. They are first clipped,
    then scaled by the residual bounds, then added to the frozen baseline.
    """

    max_linear_residual: float = 0.02
    max_angular_residual: float = 0.10
    min_linear_command: float = 0.0
    max_linear_command: float = 0.20
    max_angular_command: float = 0.60

    def __post_init__(self):
        values = (
            self.max_linear_residual,
            self.max_angular_residual,
            self.min_linear_command,
            self.max_linear_command,
            self.max_angular_command,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("action limits must be finite")
        if (
            self.max_linear_residual < 0.0 or
            self.max_angular_residual < 0.0 or
            self.min_linear_command < 0.0 or
            self.max_linear_command < self.min_linear_command or
            self.max_angular_command < 0.0
        ):
            raise ValueError("action limits are outside their valid range")


@dataclass(frozen=True)
class ResidualApplication:
    """Result of the residual safety envelope."""

    command: Command
    residual: Command
    normalized_action: Tuple[float, float]
    valid_action: bool
    clipped: bool
    fallback_reason: str = ""


def _parse_action(action):
    try:
        values = tuple(action)
    except (TypeError, ValueError):
        return None
    if len(values) != 2:
        return None
    try:
        parsed = (float(values[0]), float(values[1]))
    except (TypeError, ValueError, OverflowError):
        return None
    if not all(math.isfinite(value) for value in parsed):
        return None
    return parsed


def apply_residual_action(
    baseline: Command,
    action: Iterable[float],
    limits: ActionLimits = ActionLimits(),
    enabled: bool = True,
) -> ResidualApplication:
    """Apply a normalized residual without allowing policy safety bypass.

    A zero action returns the finite baseline bit-for-bit. A malformed or
    non-finite action also returns the finite baseline and reports a fallback.
    A non-finite baseline fails closed to a zero command because no safe
    baseline exists to preserve. The downstream deterministic Safety Gate is
    still required and remains the sole publisher of the final ``/cmd_vel``.
    """

    if not baseline.finite:
        return ResidualApplication(
            command=Command(0.0, 0.0),
            residual=Command(0.0, 0.0),
            normalized_action=(0.0, 0.0),
            valid_action=False,
            clipped=False,
            fallback_reason="nonfinite_baseline",
        )

    if (
        baseline.linear < limits.min_linear_command or
        baseline.linear > limits.max_linear_command or
        abs(baseline.angular) > limits.max_angular_command
    ):
        # Do not let absolute clipping turn a small requested residual into a
        # large correction. Preserve the baseline and defer to Safety Gate.
        return ResidualApplication(
            command=baseline,
            residual=Command(0.0, 0.0),
            normalized_action=(0.0, 0.0),
            valid_action=False,
            clipped=False,
            fallback_reason="baseline_out_of_bounds",
        )

    if not enabled:
        return ResidualApplication(
            command=baseline,
            residual=Command(0.0, 0.0),
            normalized_action=(0.0, 0.0),
            valid_action=True,
            clipped=False,
            fallback_reason="disabled",
        )

    parsed = _parse_action(action)
    if parsed is None:
        return ResidualApplication(
            command=baseline,
            residual=Command(0.0, 0.0),
            normalized_action=(0.0, 0.0),
            valid_action=False,
            clipped=False,
            fallback_reason="invalid_action",
        )

    normalized_linear = _clamp(parsed[0], -1.0, 1.0)
    normalized_angular = _clamp(parsed[1], -1.0, 1.0)
    normalized = (normalized_linear, normalized_angular)
    residual = Command(
        normalized_linear * limits.max_linear_residual,
        normalized_angular * limits.max_angular_residual,
    )

    # Preserve strict identity for the most important fallback/control case.
    if normalized == (0.0, 0.0):
        return ResidualApplication(
            command=baseline,
            residual=residual,
            normalized_action=normalized,
            valid_action=True,
            clipped=parsed != normalized,
        )

    unconstrained_linear = baseline.linear + residual.linear
    unconstrained_angular = baseline.angular + residual.angular
    command = Command(
        _clamp(
            unconstrained_linear,
            limits.min_linear_command,
            limits.max_linear_command,
        ),
        _clamp(
            unconstrained_angular,
            -limits.max_angular_command,
            limits.max_angular_command,
        ),
    )
    clipped = (
        parsed != normalized or
        command.linear != unconstrained_linear or
        command.angular != unconstrained_angular
    )
    return ResidualApplication(
        command=command,
        residual=Command(
            command.linear - baseline.linear,
            command.angular - baseline.angular,
        ),
        normalized_action=normalized,
        valid_action=True,
        clipped=clipped,
    )
