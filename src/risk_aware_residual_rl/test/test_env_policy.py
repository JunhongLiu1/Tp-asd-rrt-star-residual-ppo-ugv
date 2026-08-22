import importlib.util
import math

import pytest

from risk_aware_residual_rl.action import Command
from risk_aware_residual_rl.core_env import CoreEnvConfig
from risk_aware_residual_rl.core_env import ResidualControlCoreEnv
from risk_aware_residual_rl.errors import OptionalDependencyError
from risk_aware_residual_rl.gym_compat import make_gym_env
from risk_aware_residual_rl.observation import Observation
from risk_aware_residual_rl.policy import CallableResidualPolicy
from risk_aware_residual_rl.policy import load_policy
from risk_aware_residual_rl.policy import SafeResidualController
from risk_aware_residual_rl.policy import ZeroResidualPolicy
from risk_aware_residual_rl.training import dependency_status
from risk_aware_residual_rl.training import build_training_manifest
from risk_aware_residual_rl.training import load_config
from risk_aware_residual_rl.training import main as training_main
from risk_aware_residual_rl.training import train


def make_observation(**overrides):
    values = {
        "baseline_linear": 0.08,
        "actual_linear": 0.07,
        "baseline_angular": 0.0,
        "actual_angular": 0.0,
        "lateral_error": 0.0,
        "heading_error": 0.0,
        "curvature": 0.0,
        "goal_distance": 1.0,
        "radiation_dose_rate": 0.1,
        "terrain_impedance": 10.0,
    }
    values.update(overrides)
    return Observation(**values)


def test_same_seed_produces_identical_episode_prefix():
    first = ResidualControlCoreEnv()
    second = ResidualControlCoreEnv()
    assert first.reset(seed=71) == second.reset(seed=71)
    actions = ((0.0, 0.0), (0.5, -0.2), (-0.25, 0.4))
    for action in actions:
        assert first.step(action) == second.step(action)


def test_invalid_action_uses_baseline_and_terminates_training_episode():
    environment = ResidualControlCoreEnv()
    environment.reset(seed=31)
    unused_observation, reward, terminated, truncated, info = (
        environment.step((math.nan, 0.0))
    )
    del unused_observation
    assert info["command"] == info["baseline"]
    assert not info["valid_action"]
    assert reward == -100.0
    assert terminated
    assert not truncated


def test_time_limit_truncates_deterministically():
    environment = ResidualControlCoreEnv(
        config=CoreEnvConfig(max_steps=1))
    environment.reset(seed=1)
    unused_observation, unused_reward, terminated, truncated, info = (
        environment.step((0.0, 0.0))
    )
    del unused_observation, unused_reward
    assert not terminated
    assert truncated
    assert info["reason"] == "time_limit"


def test_environment_dynamics_and_reward_are_left_right_symmetric():
    left = ResidualControlCoreEnv()
    right = ResidualControlCoreEnv()
    left.reset(seed=1)
    right.reset(seed=1)
    for environment, sign in ((left, 1.0), (right, -1.0)):
        environment.goal_distance = 1.2
        environment.actual_linear = 0.05
        environment.actual_angular = sign * 0.03
        environment.heading_error = sign * 0.20
        environment.lateral_error = sign * 0.10
        environment.curvature = sign * 0.50
        environment.radiation_dose_rate = 0.20
        environment.terrain_impedance = 20.0
        environment.previous_linear_residual = 0.005
        environment.previous_angular_residual = sign * 0.02

    left_step = left.step((0.25, 0.40))
    right_step = right.step((0.25, -0.40))
    assert math.isclose(left_step[1], right_step[1])
    assert math.isclose(
        left_step[4]["command"].linear,
        right_step[4]["command"].linear,
    )
    assert math.isclose(
        left_step[4]["command"].angular,
        -right_step[4]["command"].angular,
    )
    assert math.isclose(left.heading_error, -right.heading_error)
    assert math.isclose(left.lateral_error, -right.lateral_error)


def test_zero_policy_is_strict_pid_baseline():
    baseline = Command(0.08, -0.10)
    controller = SafeResidualController(ZeroResidualPolicy())
    decision = controller.command(baseline, make_observation())
    assert decision.application.command is baseline
    assert decision.application.residual == Command(0.0, 0.0)


def test_policy_loading_never_creates_an_uncheckpointed_random_policy():
    assert isinstance(load_policy("zero", ""), ZeroResidualPolicy)
    with pytest.raises(ValueError) as missing_checkpoint:
        load_policy("sb3", "")
    assert "explicit checkpoint_path" in str(missing_checkpoint.value)
    with pytest.raises(ValueError) as random_policy:
        load_policy("random", "")
    assert "random policies are not permitted" in str(random_policy.value)


def test_policy_exception_and_bad_observation_fail_back_to_baseline():
    baseline = Command(0.08, 0.0)

    def failing_policy(unused_observation, unused_deterministic):
        raise RuntimeError("deliberate test failure")

    controller = SafeResidualController(
        CallableResidualPolicy(failing_policy))
    failed_policy = controller.command(baseline, make_observation())
    assert failed_policy.application.command is baseline
    assert failed_policy.policy_error == "policy_exception:RuntimeError"

    invalid_observation = controller.command(
        baseline, make_observation(actual_linear=float("nan")))
    assert invalid_observation.application.command is baseline
    assert not invalid_observation.observation_valid


def test_optional_dependency_status_matches_environment():
    status = dependency_status()
    for name, available in status.items():
        assert available == (importlib.util.find_spec(name) is not None)


def test_gym_adapter_has_clear_optional_dependency_behavior():
    if importlib.util.find_spec("gymnasium") is None:
        with pytest.raises(OptionalDependencyError) as error:
            make_gym_env()
        assert "gymnasium" in str(error.value)
    else:
        environment = make_gym_env()
        observation, unused_info = environment.reset(seed=5)
        assert environment.observation_space.contains(observation)
        environment.close()


def test_training_config_and_dry_run_need_no_rl_dependencies(tmp_path):
    config = load_config(overrides={"seed": 101, "total_timesteps": 256})
    assert config.seed == 101
    assert config.total_timesteps == 256
    assert training_main([
        "--dry-run",
        "--seed", "101",
        "--total-timesteps", "256",
        "--checkpoint-dir", str(tmp_path),
    ]) == 0
    assert not list(tmp_path.iterdir())


def test_training_manifest_records_non_deployable_environment_contract():
    config = load_config(overrides={"total_timesteps": 256})
    manifest = build_training_manifest(config)
    assert manifest["schema_version"] == 1
    assert not manifest["deployable"]
    assert manifest["environment"]["max_steps"] == 400
    assert manifest["environment"]["minimum_tracking_speed"] == 0.04
    assert manifest["training"]["total_timesteps"] == 256


def test_training_has_clear_error_when_optional_dependencies_are_missing():
    if not all(dependency_status().values()):
        with pytest.raises(OptionalDependencyError) as error:
            train(load_config(overrides={"total_timesteps": 256}))
        assert "dependencies are unavailable" in str(error.value)
