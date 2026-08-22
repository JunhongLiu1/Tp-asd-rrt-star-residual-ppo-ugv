import math

from risk_aware_residual_rl.action import ActionLimits
from risk_aware_residual_rl.action import Command
from risk_aware_residual_rl.action import apply_residual_action


def test_zero_residual_is_strict_baseline_identity():
    baseline = Command(0.08, -0.15)
    result = apply_residual_action(baseline, (0.0, 0.0))
    assert result.command is baseline
    assert result.command == baseline
    assert result.residual == Command(0.0, 0.0)
    assert result.valid_action
    assert not result.clipped


def test_residual_and_absolute_commands_are_bounded():
    limits = ActionLimits(
        max_linear_residual=0.02,
        max_angular_residual=0.10,
        max_linear_command=0.20,
        max_angular_command=0.60,
    )
    result = apply_residual_action(Command(0.19, 0.58), (10.0, 2.0), limits)
    assert result.command == Command(0.20, 0.60)
    assert result.normalized_action == (1.0, 1.0)
    assert abs(result.residual.linear) <= limits.max_linear_residual
    assert abs(result.residual.angular) <= limits.max_angular_residual
    assert result.clipped


def test_negative_residual_cannot_command_reverse():
    result = apply_residual_action(Command(0.005, 0.0), (-1.0, 0.0))
    assert result.command.linear == 0.0
    assert result.clipped


def test_nonfinite_or_malformed_action_falls_back_to_baseline():
    baseline = Command(0.07, 0.12)
    for action in ((math.nan, 0.0), (math.inf, 0.0), (1.0,), None):
        result = apply_residual_action(baseline, action)
        assert result.command is baseline
        assert not result.valid_action
        assert result.fallback_reason == "invalid_action"


def test_nonfinite_baseline_fails_closed_to_zero():
    result = apply_residual_action(Command(math.nan, 0.0), (0.0, 0.0))
    assert result.command == Command(0.0, 0.0)
    assert not result.valid_action
    assert result.fallback_reason == "nonfinite_baseline"


def test_out_of_bounds_baseline_gets_no_learned_correction():
    baseline = Command(0.30, 0.0)
    result = apply_residual_action(baseline, (-1.0, 0.5))
    assert result.command is baseline
    assert result.residual == Command(0.0, 0.0)
    assert not result.valid_action
    assert result.fallback_reason == "baseline_out_of_bounds"


def test_disabled_residual_returns_baseline_for_any_action():
    baseline = Command(0.09, -0.2)
    result = apply_residual_action(
        baseline, (0.8, -0.9), enabled=False)
    assert result.command is baseline
    assert result.residual == Command(0.0, 0.0)
    assert result.fallback_reason == "disabled"
