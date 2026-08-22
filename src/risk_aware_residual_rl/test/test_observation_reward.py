import math

from risk_aware_residual_rl.observation import OBSERVATION_FIELDS
from risk_aware_residual_rl.observation import Observation
from risk_aware_residual_rl.observation import encode_observation
from risk_aware_residual_rl.reward import RewardConfig
from risk_aware_residual_rl.reward import TransitionMetrics
from risk_aware_residual_rl.reward import calculate_reward


def make_observation(**overrides):
    values = {
        "baseline_linear": 0.10,
        "actual_linear": 0.08,
        "baseline_angular": 0.30,
        "actual_angular": 0.20,
        "lateral_error": 0.15,
        "heading_error": -0.20,
        "curvature": 0.50,
        "goal_distance": 2.0,
        "radiation_dose_rate": 0.25,
        "terrain_impedance": 30.0,
    }
    values.update(overrides)
    return Observation(**values)


def make_transition(**overrides):
    values = {
        "previous_goal_distance": 2.0,
        "goal_distance": 1.95,
        "lateral_error": 0.05,
        "heading_error": 0.10,
        "baseline_linear": 0.08,
        "actual_linear": 0.07,
        "linear_residual": 0.005,
        "angular_residual": -0.01,
    }
    values.update(overrides)
    return TransitionMetrics(**values)


def test_observation_has_stable_bounded_layout():
    result = encode_observation(make_observation())
    assert result.valid
    assert len(result.vector) == len(OBSERVATION_FIELDS)
    assert all(-1.0 <= value <= 1.0 for value in result.vector)
    linear_error_index = OBSERVATION_FIELDS.index("linear_tracking_error")
    assert math.isclose(result.vector[linear_error_index], 0.10)


def test_nonfinite_observation_is_invalid_and_zeroed():
    result = encode_observation(
        make_observation(actual_linear=float("nan")))
    assert not result.valid
    assert result.fallback_reason == "nonfinite_observation"
    assert result.vector == (0.0,) * len(OBSERVATION_FIELDS)


def test_surrogate_operating_ranges_are_not_overcompressed():
    result = encode_observation(make_observation(
        heading_error=0.30,
        goal_distance=1.80,
        radiation_dose_rate=0.40,
        terrain_impedance=45.0,
    ))
    assert result.valid
    assert math.isclose(
        result.vector[OBSERVATION_FIELDS.index("heading_error")], 0.30
    )
    assert math.isclose(
        result.vector[OBSERVATION_FIELDS.index("goal_distance")], 0.90
    )
    assert math.isclose(
        result.vector[OBSERVATION_FIELDS.index("radiation_dose_rate")], 0.80
    )
    assert math.isclose(
        result.vector[OBSERVATION_FIELDS.index("terrain_impedance")], 0.90
    )


def test_goal_reward_terminates_with_bonus():
    running = calculate_reward(make_transition())
    goal = calculate_reward(make_transition(goal_reached=True))
    assert not running.terminated
    assert goal.terminated
    assert not goal.truncated
    assert goal.reason == "goal_reached"
    assert goal.reward > running.reward


def test_every_safety_flag_gets_dominant_terminal_penalty():
    config = RewardConfig(safety_penalty=123.0)
    for flag in (
        "safety_stop", "collision", "out_of_bounds", "invalid_action"
    ):
        result = calculate_reward(make_transition(**{flag: True}), config)
        assert result.reward == -123.0
        assert result.terminated
        assert not result.truncated
        assert result.reason == "safety_violation"

    safety_at_goal = calculate_reward(make_transition(
        goal_reached=True, safety_stop=True), config)
    assert safety_at_goal.reward == -123.0
    assert safety_at_goal.reason == "safety_violation"


def test_nonfinite_transition_fails_terminal():
    result = calculate_reward(
        make_transition(goal_distance=float("inf")))
    assert result.reward == -100.0
    assert result.terminated
    assert result.reason == "nonfinite_transition"


def test_time_limit_is_truncation_not_success_or_safety_failure():
    result = calculate_reward(make_transition(time_limit_reached=True))
    assert not result.terminated
    assert result.truncated
    assert result.reason == "time_limit"


def test_saturation_and_residual_changes_reduce_reward():
    nominal = calculate_reward(make_transition())
    penalized = calculate_reward(make_transition(
        saturated=True,
        linear_residual=0.02,
        angular_residual=0.10,
        previous_linear_residual=-0.02,
        previous_angular_residual=-0.10,
    ))
    assert penalized.reward < nominal.reward


def test_more_safe_progress_increases_reward_all_else_equal():
    slower = calculate_reward(make_transition(goal_distance=1.98))
    faster = calculate_reward(make_transition(goal_distance=1.90))
    assert faster.reward > slower.reward


def test_reward_components_sum_exactly_and_keep_safety_dominant():
    nominal = calculate_reward(make_transition(
        saturated=True, goal_reached=True))
    assert math.isclose(nominal.reward, sum(nominal.components.values()))
    assert nominal.components["progress"] > 0.0
    assert nominal.components["lateral_error"] < 0.0
    assert nominal.components["goal_bonus"] == 25.0
    assert nominal.components["saturation"] == -0.5

    safety = calculate_reward(make_transition(
        safety_stop=True, goal_reached=True))
    assert safety.reward == -100.0
    assert safety.components["safety"] == -100.0
    assert safety.components["goal_bonus"] == 0.0
    assert sum(safety.components.values()) == safety.reward
