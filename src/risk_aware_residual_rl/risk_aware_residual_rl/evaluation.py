"""Deterministic offline evaluation for an explicit PPO checkpoint."""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

from .errors import OptionalDependencyError
from .gym_compat import make_gym_env
from .policy import load_policy
from .policy import ZeroResidualPolicy


ARTIFACT_LABEL = "SMOKE_ONLY_NOT_DEPLOYABLE"


class EvaluationFailure(RuntimeError):
    """Fail an evaluation when its output cannot satisfy the contract."""


@dataclass(frozen=True)
class EvaluationConfig:
    """Seed and episode limits for reproducible offline evaluation."""

    episodes: int = 10
    seed: int = 31
    deterministic: bool = True
    max_steps_per_episode: int = 400

    def __post_init__(self):
        if self.episodes <= 0:
            raise ValueError("episodes must be positive")
        if self.seed < 0:
            raise ValueError("seed must be non-negative")
        if self.max_steps_per_episode <= 0:
            raise ValueError("max_steps_per_episode must be positive")


@dataclass(frozen=True)
class AcceptanceCriteria:
    """Minimum A/B gates required for surrogate checkpoint acceptance."""

    min_success_rate: float = 0.95
    max_time_limit_rate: float = 0.05
    max_success_rate_drop: float = 0.02

    def __post_init__(self):
        values = (
            self.min_success_rate,
            self.max_time_limit_rate,
            self.max_success_rate_drop,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("acceptance rates must be finite")
        if not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError("acceptance rates must be within [0, 1]")


def validate_action(action):
    """Return a finite two-value action or fail before stepping the env."""
    try:
        values = tuple(action)
    except (TypeError, ValueError) as exception:
        raise EvaluationFailure("policy action is not iterable") from exception
    if len(values) != 2:
        raise EvaluationFailure("policy action must contain exactly 2 values")
    try:
        parsed = (float(values[0]), float(values[1]))
    except (TypeError, ValueError, OverflowError) as exception:
        raise EvaluationFailure("policy action is not numeric") from exception
    if not all(math.isfinite(value) for value in parsed):
        raise EvaluationFailure("policy action contains a non-finite value")
    if any(abs(value) > 1.0 for value in parsed):
        raise EvaluationFailure("policy action is outside normalized bounds")
    return parsed


def evaluate_policy(policy, environment, config=EvaluationConfig()):
    """Evaluate a policy with hard finite/bounds checks on every action."""
    episode_returns = []
    episode_lengths = []
    terminated_episodes = 0
    truncated_episodes = 0
    actions_checked = 0
    max_abs_action = 0.0
    termination_reasons = {}
    reward_component_totals = {}

    for episode in range(config.episodes):
        observation, unused_info = environment.reset(
            seed=config.seed + episode)
        del unused_info
        episode_return = 0.0
        for step in range(1, config.max_steps_per_episode + 1):
            try:
                prediction = policy.predict(
                    observation, deterministic=config.deterministic)
            except Exception as exception:
                raise EvaluationFailure(
                    "policy prediction failed: " +
                    exception.__class__.__name__
                ) from exception
            action = validate_action(prediction)
            actions_checked += 1
            max_abs_action = max(
                max_abs_action, abs(action[0]), abs(action[1]))
            observation, reward, terminated, truncated, info = (
                environment.step(action)
            )
            reward = float(reward)
            if not math.isfinite(reward):
                raise EvaluationFailure(
                    "environment returned non-finite reward")
            for name, contribution in info.get(
                "reward_components", {}
            ).items():
                contribution = float(contribution)
                if not math.isfinite(contribution):
                    raise EvaluationFailure(
                        "reward component is non-finite: " + str(name)
                    )
                reward_component_totals[name] = (
                    reward_component_totals.get(name, 0.0) + contribution
                )
            episode_return += reward
            if terminated or truncated:
                reason = str(info.get("reason", "")).strip()
                if not reason:
                    reason = "terminated" if terminated else "truncated"
                termination_reasons[reason] = (
                    termination_reasons.get(reason, 0) + 1
                )
                terminated_episodes += int(bool(terminated))
                truncated_episodes += int(bool(truncated))
                episode_returns.append(episode_return)
                episode_lengths.append(step)
                break
        else:
            raise EvaluationFailure(
                "environment did not finish within max_steps_per_episode"
            )

    goal_reached = termination_reasons.get("goal_reached", 0)
    time_limits = termination_reasons.get("time_limit", 0)
    safety_terminations = sum(
        termination_reasons.get(reason, 0)
        for reason in ("safety_violation", "nonfinite_transition")
    )
    return {
        "acceptance_passed": None,
        "action_checks": {
            "actions_checked": actions_checked,
            "bounds_passed": True,
            "finite_passed": True,
            "max_abs_action": max_abs_action,
        },
        "deterministic": config.deterministic,
        "episode_lengths": episode_lengths,
        "episode_returns": episode_returns,
        "episodes": config.episodes,
        "execution_valid": True,
        "goal_reached": goal_reached,
        "mean_episode_length": sum(episode_lengths) / len(episode_lengths),
        "mean_return": sum(episode_returns) / len(episode_returns),
        "reward_components_mean_per_episode": {
            name: total / config.episodes
            for name, total in sorted(reward_component_totals.items())
        },
        "safety_terminations": safety_terminations,
        "seed": config.seed,
        "success_rate": goal_reached / config.episodes,
        "termination_reasons": termination_reasons,
        "terminated_episodes": terminated_episodes,
        "time_limit_rate": time_limits / config.episodes,
        "truncated_episodes": truncated_episodes,
    }


def compare_against_zero_baseline(
    candidate, zero_baseline, criteria=AcceptanceCriteria()
):
    """Apply the documented same-seed surrogate acceptance gates."""
    checks = {
        "higher_mean_return_than_zero": (
            candidate["mean_return"] > zero_baseline["mean_return"]
        ),
        "lower_mean_episode_length_than_zero": (
            candidate["mean_episode_length"] <
            zero_baseline["mean_episode_length"]
        ),
        "no_safety_terminations": candidate["safety_terminations"] == 0,
        "success_drop_within_limit": (
            candidate["success_rate"] >=
            zero_baseline["success_rate"] -
            criteria.max_success_rate_drop
        ),
        "success_rate_at_least_minimum": (
            candidate["success_rate"] >= criteria.min_success_rate
        ),
        "time_limit_rate_at_most_maximum": (
            candidate["time_limit_rate"] <= criteria.max_time_limit_rate
        ),
    }
    return {
        "acceptance_passed": all(checks.values()),
        "checks": checks,
        "criteria": {
            "max_success_rate_drop": criteria.max_success_rate_drop,
            "max_time_limit_rate": criteria.max_time_limit_rate,
            "min_success_rate": criteria.min_success_rate,
        },
    }


def _emit_result(result, output_path=None):
    serialized = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if output_path:
        path = Path(output_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(serialized, encoding="utf-8")
    print(serialized, end="")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Evaluate one explicit PPO checkpoint offline."
    )
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--seed", type=int, default=31)
    parser.add_argument("--max-steps", type=int, default=400)
    parser.add_argument("--min-success-rate", type=float, default=0.95)
    parser.add_argument("--max-time-limit-rate", type=float, default=0.05)
    parser.add_argument("--max-success-rate-drop", type=float, default=0.02)
    parser.add_argument(
        "--no-zero-baseline",
        action="store_true",
        help="run execution checks only; acceptance_passed remains null",
    )
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="request stochastic prediction (deterministic is the default)",
    )
    parser.add_argument("--output", help="optional JSON output path")
    return parser.parse_args(argv)


def main(
    argv=None,
    policy_loader=load_policy,
    environment_factory=make_gym_env,
):
    args = parse_args(argv)
    result = {
        "artifact_label": ARTIFACT_LABEL,
        "acceptance_passed": False,
        "checkpoint": str(Path(args.checkpoint).expanduser()),
        "deployable": False,
        "execution_valid": False,
    }
    environments = []
    try:
        config = EvaluationConfig(
            episodes=args.episodes,
            seed=args.seed,
            deterministic=not args.stochastic,
            max_steps_per_episode=args.max_steps,
        )
        policy = policy_loader("sb3", args.checkpoint)
        candidate_environment = environment_factory()
        environments.append(candidate_environment)
        candidate = evaluate_policy(policy, candidate_environment, config)
        result["candidate"] = candidate
        result["execution_valid"] = True
        if args.no_zero_baseline:
            result["acceptance_passed"] = None
            result["acceptance"] = {
                "reason": "zero_baseline_comparison_disabled"
            }
        else:
            baseline_environment = environment_factory()
            environments.append(baseline_environment)
            zero_baseline = evaluate_policy(
                ZeroResidualPolicy(), baseline_environment, config
            )
            result["zero_baseline"] = zero_baseline
            acceptance = compare_against_zero_baseline(
                candidate,
                zero_baseline,
                AcceptanceCriteria(
                    min_success_rate=args.min_success_rate,
                    max_time_limit_rate=args.max_time_limit_rate,
                    max_success_rate_drop=args.max_success_rate_drop,
                ),
            )
            result["acceptance"] = acceptance
            result["acceptance_passed"] = acceptance[
                "acceptance_passed"
            ]
        _emit_result(result, args.output)
        return 0 if result["acceptance_passed"] is not False else 3
    except (
        EvaluationFailure,
        OptionalDependencyError,
        OSError,
        ValueError,
    ) as exception:
        result["execution_valid"] = False
        result["acceptance_passed"] = False
        result["error"] = str(exception)
        _emit_result(result, args.output)
        return 2
    finally:
        for environment in environments:
            environment.close()


if __name__ == "__main__":
    sys.exit(main())
