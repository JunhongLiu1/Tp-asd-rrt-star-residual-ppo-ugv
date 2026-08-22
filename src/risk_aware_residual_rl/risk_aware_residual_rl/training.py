"""Optional Stable-Baselines3 PPO training command line."""

import argparse
from dataclasses import asdict
from dataclasses import dataclass
import importlib.util
import json
import math
from pathlib import Path
import random
import sys

from .action import ActionLimits
from .core_env import CoreEnvConfig
from .errors import OptionalDependencyError
from .gym_compat import make_gym_env
from .observation import ObservationConfig
from .observation import OBSERVATION_FIELDS
from .reward import RewardConfig
from .worker_supervisor import sha256_file


ARTIFACT_LABEL = "TRAINING_OUTPUT_NOT_DEPLOYMENT_APPROVED"


@dataclass(frozen=True)
class TrainingConfig:
    """Small reproducible PPO configuration suitable for first smoke runs."""

    seed: int = 31
    total_timesteps: int = 10000
    learning_rate: float = 0.0003
    n_steps: int = 256
    batch_size: int = 64
    n_epochs: int = 10
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.20
    ent_coef: float = 0.0
    checkpoint_freq: int = 5000
    checkpoint_dir: str = "checkpoints/residual_ppo"
    device: str = "auto"

    def __post_init__(self):
        if self.seed < 0:
            raise ValueError("seed must be non-negative")
        if (
            self.total_timesteps <= 0 or
            self.n_steps <= 0 or
            self.batch_size <= 0 or
            self.n_epochs <= 0 or
            self.checkpoint_freq <= 0
        ):
            raise ValueError("training counts must be positive")
        if self.n_steps % self.batch_size != 0:
            raise ValueError("n_steps must be divisible by batch_size")
        continuous = (
            self.learning_rate,
            self.gamma,
            self.gae_lambda,
            self.clip_range,
            self.ent_coef,
        )
        if not all(math.isfinite(value) for value in continuous):
            raise ValueError("training hyperparameters must be finite")
        if not 0.0 < self.learning_rate:
            raise ValueError("learning_rate must be positive")
        if not 0.0 < self.gamma <= 1.0:
            raise ValueError("gamma must be in (0, 1]")
        if not 0.0 < self.gae_lambda <= 1.0:
            raise ValueError("gae_lambda must be in (0, 1]")
        if not 0.0 < self.clip_range < 1.0:
            raise ValueError("clip_range must be in (0, 1)")
        if self.ent_coef < 0.0:
            raise ValueError("ent_coef must be non-negative")


def dependency_status():
    """Return availability without importing heavyweight optional modules."""
    return {
        name: importlib.util.find_spec(name) is not None
        for name in ("gymnasium", "stable_baselines3", "torch")
    }


def load_config(path=None, overrides=None):
    """Load a JSON configuration and apply non-None CLI overrides."""
    values = asdict(TrainingConfig())
    if path:
        with Path(path).open("r", encoding="utf-8") as stream:
            loaded = json.load(stream)
        unknown = sorted(set(loaded) - set(values))
        if unknown:
            raise ValueError(
                "unknown training configuration keys: " + ", ".join(unknown)
            )
        values.update(loaded)
    for key, value in (overrides or {}).items():
        if value is not None:
            values[key] = value
    return TrainingConfig(**values)


def seed_everything(seed):
    """Seed available random generators without requiring optional packages."""
    random.seed(seed)
    try:
        import numpy
        numpy.random.seed(seed)
    except ImportError:
        pass
    try:
        import torch
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
    except ImportError:
        pass


def build_training_manifest(config, checkpoint_sha256=""):
    """Capture every default contract used by the surrogate training env."""
    return {
        "action_limits": asdict(ActionLimits()),
        "action_contract": {
            "normalized_bounds": [-1.0, 1.0],
            "shape": [2],
        },
        "artifact_label": ARTIFACT_LABEL,
        "deployable": False,
        "checkpoint_sha256": checkpoint_sha256,
        "environment": asdict(CoreEnvConfig()),
        "observation": asdict(ObservationConfig()),
        "observation_fields": list(OBSERVATION_FIELDS),
        "reward": asdict(RewardConfig()),
        "schema_version": 1,
        "training": asdict(config),
    }


def train(config):
    """Train PPO only when all optional dependencies were installed by user."""
    status = dependency_status()
    missing = [name for name, available in status.items() if not available]
    if missing:
        raise OptionalDependencyError(
            "PPO training dependencies are unavailable: " +
            ", ".join(missing) +
            ". Install compatible versions explicitly; this package does "
            "not download them."
        )

    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback

    seed_everything(config.seed)
    environment = make_gym_env()
    checkpoint_dir = Path(config.checkpoint_dir).expanduser().resolve()
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    callback = CheckpointCallback(
        save_freq=config.checkpoint_freq,
        save_path=str(checkpoint_dir),
        name_prefix="residual_ppo",
    )
    model = PPO(
        "MlpPolicy",
        environment,
        learning_rate=config.learning_rate,
        n_steps=config.n_steps,
        batch_size=config.batch_size,
        n_epochs=config.n_epochs,
        gamma=config.gamma,
        gae_lambda=config.gae_lambda,
        clip_range=config.clip_range,
        ent_coef=config.ent_coef,
        seed=config.seed,
        device=config.device,
        verbose=1,
    )
    model.learn(total_timesteps=config.total_timesteps, callback=callback)
    final_checkpoint = checkpoint_dir / "residual_ppo_final"
    model.save(str(final_checkpoint))
    final_checkpoint_path = str(final_checkpoint) + ".zip"
    with (checkpoint_dir / "training_config.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(asdict(config), stream, indent=2, sort_keys=True)
        stream.write("\n")
    with (checkpoint_dir / "training_manifest.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(
            build_training_manifest(
                config, sha256_file(final_checkpoint_path)
            ),
            stream,
            indent=2,
            sort_keys=True,
        )
        stream.write("\n")
    environment.close()
    return final_checkpoint_path


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Train bounded residual PPO on the ROS-free smoke env."
    )
    parser.add_argument("--config", help="JSON training configuration")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--total-timesteps", type=int)
    parser.add_argument("--checkpoint-freq", type=int)
    parser.add_argument("--checkpoint-dir")
    parser.add_argument("--device")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate config and report dependencies without training",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        config = load_config(
            args.config,
            {
                "seed": args.seed,
                "total_timesteps": args.total_timesteps,
                "checkpoint_freq": args.checkpoint_freq,
                "checkpoint_dir": args.checkpoint_dir,
                "device": args.device,
            },
        )
        if args.dry_run:
            print(json.dumps({
                "config": asdict(config),
                "optional_dependencies": dependency_status(),
                "training_started": False,
            }, indent=2, sort_keys=True))
            return 0
        checkpoint = train(config)
        print("Training complete; checkpoint: " + checkpoint)
        return 0
    except (ValueError, OSError, json.JSONDecodeError,
            OptionalDependencyError) as exception:
        print("ERROR: " + str(exception), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
