"""Audit and finalize a trained PPO artifact for isolated inference."""

import argparse
from dataclasses import asdict
import importlib.metadata
import json
import math
import os
from pathlib import Path
import sys

from .training import build_training_manifest
from .training import load_config
from .worker_supervisor import sha256_file
from .worker_supervisor import validate_artifact_contract


FINALIZATION_SCHEMA_VERSION = 1


def _flatten(values):
    if hasattr(values, "reshape"):
        values = values.reshape(-1).tolist()
    if not isinstance(values, (list, tuple)):
        return [float(values)]
    result = []
    for value in values:
        if isinstance(value, (list, tuple)):
            result.extend(_flatten(value))
        else:
            result.append(float(value))
    return result


def summarize_space(space):
    """Convert a continuous Gym space into auditable JSON primitives."""
    try:
        shape = [int(value) for value in space.shape]
        low = _flatten(space.low)
        high = _flatten(space.high)
        dtype = str(space.dtype)
    except (AttributeError, TypeError, ValueError) as exception:
        raise ValueError("model/environment space is not a supported Box") \
            from exception
    if not low or len(low) != len(high):
        raise ValueError("model/environment space bounds are malformed")
    if not all(math.isfinite(value) for value in low + high):
        raise ValueError("model/environment space bounds are non-finite")
    return {"dtype": dtype, "high": high, "low": low, "shape": shape}


def _require_equal(name, actual, expected):
    if actual != expected:
        raise ValueError(
            "checkpoint {} mismatch: actual={!r}, expected={!r}".format(
                name, actual, expected
            )
        )


def _require_float(name, actual, expected):
    if not math.isclose(
        float(actual), float(expected), rel_tol=1e-12, abs_tol=1e-12
    ):
        raise ValueError(
            "checkpoint {} mismatch: actual={!r}, expected={!r}".format(
                name, actual, expected
            )
        )


def audit_loaded_model(model, environment, config):
    """Prove saved model spaces and training metadata match this package."""
    model_observation = summarize_space(model.observation_space)
    model_action = summarize_space(model.action_space)
    environment_observation = summarize_space(environment.observation_space)
    environment_action = summarize_space(environment.action_space)
    _require_equal(
        "observation_space", model_observation, environment_observation
    )
    _require_equal("action_space", model_action, environment_action)
    _require_equal("seed", int(model.seed), config.seed)
    expected_timesteps = (
        math.ceil(config.total_timesteps / config.n_steps) * config.n_steps
    )
    _require_equal(
        "num_timesteps", int(model.num_timesteps), expected_timesteps
    )
    for name in ("n_steps", "batch_size", "n_epochs"):
        _require_equal(name, int(getattr(model, name)), getattr(config, name))
    for name in (
        "learning_rate",
        "gamma",
        "gae_lambda",
        "ent_coef",
    ):
        _require_float(name, getattr(model, name), getattr(config, name))
    clip_range = model.clip_range(1.0)
    _require_float("clip_range", clip_range, config.clip_range)
    reset_observation, unused_info = environment.reset(seed=config.seed)
    del unused_info
    if not environment.observation_space.contains(reset_observation):
        raise ValueError(
            "current environment reset violates observation space")
    return {
        "action_space": model_action,
        "actual_num_timesteps": int(model.num_timesteps),
        "configured_total_timesteps": config.total_timesteps,
        "expected_rollout_timesteps": expected_timesteps,
        "observation_space": model_observation,
        "seed": int(model.seed),
        "training_hyperparameters_match": True,
    }


def _atomic_json_write(path, payload):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))
    return path


def finalize_artifact(
    checkpoint_path,
    training_config_path,
    manifest_path,
    evidence_path,
):
    """Load, audit, finalize, then independently validate one checkpoint."""
    try:
        from stable_baselines3 import PPO
    except ImportError as exception:
        raise RuntimeError(
            "artifact finalization requires the pinned PPO dependencies"
        ) from exception
    from .gym_compat import make_gym_env

    checkpoint = Path(checkpoint_path).expanduser().resolve()
    training_config_file = Path(training_config_path).expanduser().resolve()
    if not checkpoint.is_file():
        raise ValueError("checkpoint does not exist: " + str(checkpoint))
    if not training_config_file.is_file():
        raise ValueError(
            "training config does not exist: " + str(training_config_file)
        )
    config = load_config(str(training_config_file))
    checkpoint_hash = sha256_file(checkpoint)
    config_hash = sha256_file(training_config_file)
    environment = make_gym_env()
    try:
        model = PPO.load(str(checkpoint), device="cpu")
        model_audit = audit_loaded_model(model, environment, config)
    finally:
        environment.close()
    manifest = build_training_manifest(config, checkpoint_hash)
    manifest["finalization"] = {
        "checkpoint_path": str(checkpoint),
        "checkpoint_sha256": checkpoint_hash,
        "finalization_schema_version": FINALIZATION_SCHEMA_VERSION,
        "model_audit": model_audit,
        "training_config_path": str(training_config_file),
        "training_config_sha256": config_hash,
    }
    output_manifest = _atomic_json_write(manifest_path, manifest)
    validation = validate_artifact_contract(
        checkpoint,
        output_manifest,
        checkpoint_hash,
    )
    evidence = {
        "artifact_label": "AUDITED_OFFLINE_ARTIFACT_NOT_GAZEBO_APPROVAL",
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": checkpoint_hash,
        "contract_validation_passed": True,
        "deployable": False,
        "manifest": str(output_manifest),
        "manifest_sha256": sha256_file(output_manifest),
        "model_audit": model_audit,
        "runtime_versions": {
            name: importlib.metadata.version(name)
            for name in ("gymnasium", "stable-baselines3", "torch")
        },
        "training_config": asdict(config),
        "training_config_path": str(training_config_file),
        "training_config_sha256": config_hash,
        "validator_result": validation,
    }
    output_evidence = _atomic_json_write(evidence_path, evidence)
    evidence["evidence"] = str(output_evidence)
    return evidence


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Audit PPO metadata and finalize its inference manifest."
    )
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--training-config", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--evidence", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        evidence = finalize_artifact(
            args.checkpoint,
            args.training_config,
            args.manifest,
            args.evidence,
        )
        print(json.dumps(evidence, indent=2, sort_keys=True))
        return 0
    except (
        json.JSONDecodeError,
        OSError,
        RuntimeError,
        TypeError,
        ValueError,
    ) as exception:
        print("ERROR: " + str(exception), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
