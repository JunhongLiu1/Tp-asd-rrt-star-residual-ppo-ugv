import math

from risk_aware_residual_rl.action import Command
from risk_aware_residual_rl.action import apply_residual_action
from risk_aware_residual_rl.adapter_core import AdapterConfig
from risk_aware_residual_rl.adapter_core import AdapterInputs
from risk_aware_residual_rl.adapter_core import ResidualAdapterStateMachine


def make_inputs(**overrides):
    values = {
        "now_sec": 10.0,
        "baseline": Command(0.08, 0.10),
        "baseline_received_sec": 9.9,
        "metrics_received_sec": 9.8,
        "auxiliary_inputs_finite": True,
        "enable_rl": True,
        "emergency_stop": False,
        "motion_stopped": False,
    }
    values.update(overrides)
    return AdapterInputs(**values)


def test_startup_without_baseline_outputs_zero():
    state = ResidualAdapterStateMachine()
    decision = state.evaluate(AdapterInputs(now_sec=1.0))
    assert decision.command == Command(0.0, 0.0)
    assert not decision.apply_policy
    assert decision.clear_policy_state
    assert decision.reason == "waiting_baseline"


def test_disabled_rl_is_exact_baseline_pass_through():
    state = ResidualAdapterStateMachine()
    inputs = make_inputs(enable_rl=False)
    decision = state.evaluate(inputs)
    assert decision.command is inputs.baseline
    assert not decision.apply_policy
    assert decision.reason == "rl_disabled_baseline"


def test_fresh_inputs_are_the_only_policy_permission():
    state = ResidualAdapterStateMachine()
    assert state.evaluate(make_inputs()).apply_policy

    stale_metrics = state.evaluate(make_inputs(metrics_received_sec=9.0))
    assert stale_metrics.command == make_inputs().baseline
    assert not stale_metrics.apply_policy
    assert stale_metrics.reason == "stale_metrics_baseline"

    missing_metrics = state.evaluate(make_inputs(metrics_received_sec=None))
    assert missing_metrics.command == make_inputs().baseline
    assert missing_metrics.reason == "waiting_metrics_baseline"


def test_stale_or_nonfinite_baseline_and_auxiliary_data_output_zero():
    state = ResidualAdapterStateMachine()
    cases = (
        make_inputs(baseline_received_sec=9.0),
        make_inputs(baseline=Command(math.nan, 0.0)),
        make_inputs(auxiliary_inputs_finite=False),
    )
    for inputs in cases:
        decision = state.evaluate(inputs)
        assert decision.command == Command(0.0, 0.0)
        assert not decision.apply_policy
        assert decision.clear_policy_state


def test_stop_and_estop_clear_state_and_output_zero():
    state = ResidualAdapterStateMachine()
    for inputs in (
        make_inputs(emergency_stop=True),
        make_inputs(motion_stopped=True),
    ):
        decision = state.evaluate(inputs)
        assert decision.command == Command(0.0, 0.0)
        assert decision.clear_policy_state


def test_policy_result_is_bounded_or_falls_back_on_timeout():
    state = ResidualAdapterStateMachine(
        AdapterConfig(model_timeout_sec=0.05))
    baseline = Command(0.08, 0.10)
    application = apply_residual_action(baseline, (1.0, -1.0))
    accepted = state.finalize_policy(baseline, application, 0.01)
    assert accepted.command == application.command
    assert accepted.reason == "policy_command"

    late = state.finalize_policy(baseline, application, 0.06)
    assert late.command is baseline
    assert late.reason == "model_timeout_baseline"
    assert late.clear_policy_state


def test_invalid_policy_application_falls_back_to_baseline():
    state = ResidualAdapterStateMachine()
    baseline = Command(0.08, 0.10)
    invalid = apply_residual_action(baseline, (math.nan, 0.0))
    decision = state.finalize_policy(baseline, invalid, 0.01)
    assert decision.command is baseline
    assert decision.clear_policy_state
    assert decision.reason.startswith("policy_fallback_baseline")
