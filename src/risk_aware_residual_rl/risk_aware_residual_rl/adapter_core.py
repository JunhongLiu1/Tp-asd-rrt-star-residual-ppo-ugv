"""ROS-independent state machine for the residual command adapter."""

from dataclasses import dataclass
import math
from typing import Optional

from .action import Command
from .action import ResidualApplication


@dataclass(frozen=True)
class AdapterConfig:
    """Freshness and inference limits enforced before publishing commands."""

    baseline_timeout_sec: float = 0.30
    metrics_timeout_sec: float = 0.50
    model_timeout_sec: float = 0.05

    def __post_init__(self):
        values = (
            self.baseline_timeout_sec,
            self.metrics_timeout_sec,
            self.model_timeout_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("adapter timeouts must be finite and positive")


@dataclass(frozen=True)
class AdapterInputs:
    """Latest adapter inputs using monotonic receive timestamps."""

    now_sec: float
    baseline: Optional[Command] = None
    baseline_received_sec: Optional[float] = None
    metrics_received_sec: Optional[float] = None
    auxiliary_inputs_finite: bool = True
    enable_rl: bool = False
    emergency_stop: bool = False
    motion_stopped: bool = False


@dataclass(frozen=True)
class AdapterDecision:
    """Command decision made before optional policy inference."""

    command: Command
    apply_policy: bool
    clear_policy_state: bool
    reason: str


class ResidualAdapterStateMachine:
    """Deterministic safety state machine independent of ROS and models."""

    def __init__(self, config=AdapterConfig()):
        self.config = config

    @staticmethod
    def _valid_timestamp(value):
        return value is not None and math.isfinite(value)

    def evaluate(self, inputs):
        """Select zero, baseline pass-through, or permission for inference."""
        if not math.isfinite(inputs.now_sec):
            return AdapterDecision(
                Command(0.0, 0.0), False, True, "invalid_clock")
        if inputs.emergency_stop:
            return AdapterDecision(
                Command(0.0, 0.0), False, True, "emergency_stop")
        if inputs.motion_stopped:
            return AdapterDecision(
                Command(0.0, 0.0), False, True, "motion_stopped")
        if inputs.baseline is None:
            return AdapterDecision(
                Command(0.0, 0.0), False, True, "waiting_baseline")
        if not inputs.baseline.finite:
            return AdapterDecision(
                Command(0.0, 0.0), False, True, "nonfinite_baseline")
        if not self._valid_timestamp(inputs.baseline_received_sec):
            return AdapterDecision(
                Command(0.0, 0.0), False, True, "waiting_baseline")
        baseline_age = max(
            0.0, inputs.now_sec - inputs.baseline_received_sec)
        if baseline_age > self.config.baseline_timeout_sec:
            return AdapterDecision(
                Command(0.0, 0.0), False, True, "stale_baseline")
        if not inputs.auxiliary_inputs_finite:
            return AdapterDecision(
                Command(0.0, 0.0), False, True,
                "nonfinite_auxiliary_input")
        if not inputs.enable_rl:
            return AdapterDecision(
                inputs.baseline, False, True, "rl_disabled_baseline")
        if not self._valid_timestamp(inputs.metrics_received_sec):
            return AdapterDecision(
                inputs.baseline, False, True, "waiting_metrics_baseline")
        metrics_age = max(
            0.0, inputs.now_sec - inputs.metrics_received_sec)
        if metrics_age > self.config.metrics_timeout_sec:
            return AdapterDecision(
                inputs.baseline, False, True, "stale_metrics_baseline")
        return AdapterDecision(
            inputs.baseline, True, False, "policy_permitted")

    def finalize_policy(self, baseline, application, inference_duration_sec):
        """Reject late or invalid inference without changing the baseline."""
        if (
            not math.isfinite(inference_duration_sec) or
            inference_duration_sec > self.config.model_timeout_sec
        ):
            return AdapterDecision(
                baseline, False, True, "model_timeout_baseline")
        if not isinstance(application, ResidualApplication):
            return AdapterDecision(
                baseline, False, True, "invalid_policy_result_baseline")
        if not application.valid_action:
            return AdapterDecision(
                baseline, False, True,
                "policy_fallback_baseline:" + application.fallback_reason)
        if not application.command.finite:
            return AdapterDecision(
                baseline, False, True, "nonfinite_policy_command_baseline")
        return AdapterDecision(
            application.command, False, False, "policy_command")
