import json
import math

import pytest

from risk_aware_residual_rl.core_env import ResidualControlCoreEnv
from risk_aware_residual_rl.evaluation import ARTIFACT_LABEL
from risk_aware_residual_rl.evaluation import EvaluationConfig
from risk_aware_residual_rl.evaluation import EvaluationFailure
from risk_aware_residual_rl.evaluation import compare_against_zero_baseline
from risk_aware_residual_rl.evaluation import evaluate_policy
from risk_aware_residual_rl.evaluation import main as evaluation_main
from risk_aware_residual_rl.evaluation import validate_action
from risk_aware_residual_rl.observation import OBSERVATION_FIELDS


class MockPolicy:
    def __init__(self, action=(0.25, -0.50)):
        self.action = action
        self.deterministic_values = []

    def predict(self, unused_observation, deterministic=True):
        self.deterministic_values.append(deterministic)
        return self.action


class MockEnvironment:
    def __init__(self):
        self.reset_seeds = []
        self.steps = 0
        self.closed = False

    def reset(self, seed=None):
        self.reset_seeds.append(seed)
        self.steps = 0
        return (0.0,) * 14, {}

    def step(self, unused_action):
        self.steps += 1
        terminated = self.steps == 2
        info = {
            "reason": "goal_reached" if terminated else "running",
            "reward_components": {"progress": float(self.steps)},
        }
        return (0.0,) * 14, float(self.steps), terminated, False, info

    def close(self):
        self.closed = True


def test_evaluation_is_seeded_deterministic_and_not_deployable():
    policy = MockPolicy()
    environment = MockEnvironment()
    result = evaluate_policy(
        policy,
        environment,
        EvaluationConfig(episodes=2, seed=7, deterministic=True),
    )
    assert environment.reset_seeds == [7, 8]
    assert policy.deterministic_values == [True, True, True, True]
    assert result["episode_returns"] == [3.0, 3.0]
    assert result["episode_lengths"] == [2, 2]
    assert result["action_checks"]["actions_checked"] == 4
    assert result["action_checks"]["max_abs_action"] == 0.5
    assert result["execution_valid"]
    assert result["acceptance_passed"] is None
    assert result["success_rate"] == 1.0
    assert result["reward_components_mean_per_episode"]["progress"] == 3.0


@pytest.mark.parametrize(
    "action, message",
    [
        ((math.nan, 0.0), "non-finite"),
        ((math.inf, 0.0), "non-finite"),
        ((1.0001, 0.0), "outside normalized bounds"),
        ((0.0,), "exactly 2"),
        (None, "not iterable"),
    ],
)
def test_action_validation_fails_nonfinite_or_out_of_bounds(action, message):
    with pytest.raises(EvaluationFailure) as error:
        validate_action(action)
    assert message in str(error.value)


def test_cli_writes_json_with_mock_policy_and_no_sb3(tmp_path, capsys):
    environment = MockEnvironment()
    policy = MockPolicy(action=(0.0, 0.0))
    output = tmp_path / "evaluation.json"

    def policy_loader(policy_type, checkpoint):
        assert policy_type == "sb3"
        assert checkpoint == "mock.zip"
        return policy

    return_code = evaluation_main(
        [
            "--checkpoint", "mock.zip",
            "--episodes", "1",
            "--seed", "13",
            "--no-zero-baseline",
            "--output", str(output),
        ],
        policy_loader=policy_loader,
        environment_factory=lambda: environment,
    )
    result = json.loads(output.read_text(encoding="utf-8"))
    stdout_result = json.loads(capsys.readouterr().out)
    assert return_code == 0
    assert result == stdout_result
    assert result["checkpoint"] == "mock.zip"
    assert result["artifact_label"] == ARTIFACT_LABEL
    assert not result["deployable"]
    assert result["execution_valid"]
    assert result["acceptance_passed"] is None
    assert result["candidate"]["success_rate"] == 1.0
    assert environment.closed


def test_cli_invalid_action_emits_failed_non_deployable_json(capsys):
    environment = MockEnvironment()
    return_code = evaluation_main(
        ["--checkpoint", "mock.zip", "--episodes", "1"],
        policy_loader=lambda unused_type, unused_path: MockPolicy((2.0, 0.0)),
        environment_factory=lambda: environment,
    )
    result = json.loads(capsys.readouterr().out)
    assert return_code == 2
    assert not result["execution_valid"]
    assert not result["acceptance_passed"]
    assert not result["deployable"]
    assert "outside normalized bounds" in result["error"]
    assert environment.closed


def test_cli_valid_execution_can_fail_same_seed_acceptance(capsys):
    environments = []

    def environment_factory():
        environment = MockEnvironment()
        environments.append(environment)
        return environment

    return_code = evaluation_main(
        ["--checkpoint", "mock.zip", "--episodes", "1"],
        policy_loader=lambda unused_type, unused_path: MockPolicy((0.0, 0.0)),
        environment_factory=environment_factory,
    )
    result = json.loads(capsys.readouterr().out)
    assert return_code == 3
    assert result["execution_valid"]
    assert not result["acceptance_passed"]
    assert not result["acceptance"]["checks"][
        "higher_mean_return_than_zero"
    ]
    assert len(environments) == 2
    assert all(environment.closed for environment in environments)


def test_policy_exception_becomes_evaluation_failure():
    class FailingPolicy:
        def predict(self, unused_observation, deterministic=True):
            del unused_observation, deterministic
            raise RuntimeError("deliberate mock failure")

    with pytest.raises(EvaluationFailure) as error:
        evaluate_policy(
            FailingPolicy(), MockEnvironment(), EvaluationConfig(episodes=1)
        )
    assert "policy prediction failed: RuntimeError" in str(error.value)


def test_same_seed_acceptance_requires_every_documented_gate():
    zero = {
        "mean_episode_length": 260.0,
        "mean_return": -58.0,
        "safety_terminations": 0,
        "success_rate": 1.0,
        "time_limit_rate": 0.0,
    }
    candidate = {
        "mean_episode_length": 250.0,
        "mean_return": -75.0,
        "safety_terminations": 0,
        "success_rate": 0.99,
        "time_limit_rate": 0.01,
    }
    result = compare_against_zero_baseline(candidate, zero)
    assert not result["acceptance_passed"]
    assert not result["checks"]["higher_mean_return_than_zero"]
    assert result["checks"]["lower_mean_episode_length_than_zero"]
    assert result["checks"]["success_drop_within_limit"]


def test_default_environment_is_reachable_by_zero_and_heuristic_policies():
    heading_index = OBSERVATION_FIELDS.index("heading_error")

    def zero_policy(unused_observation):
        return (0.0, 0.0)

    def heuristic_policy(observation):
        angular = max(-1.0, min(1.0, -10.0 * observation[heading_index]))
        return (1.0, angular)

    def run(policy):
        successes = 0
        lengths = []
        returns = []
        for seed in range(20):
            environment = ResidualControlCoreEnv()
            observation, unused_info = environment.reset(seed=seed)
            del unused_info
            episode_return = 0.0
            while True:
                observation, reward, terminated, truncated, info = (
                    environment.step(policy(observation))
                )
                episode_return += reward
                if terminated or truncated:
                    successes += int(info["reason"] == "goal_reached")
                    lengths.append(environment.steps)
                    returns.append(episode_return)
                    break
        return successes, sum(lengths), sum(returns)

    zero_successes, zero_steps, zero_return = run(zero_policy)
    heuristic_successes, heuristic_steps, heuristic_return = run(
        heuristic_policy
    )
    assert zero_successes >= 19
    assert heuristic_successes == 20
    assert heuristic_steps < zero_steps
    assert heuristic_return > zero_return
