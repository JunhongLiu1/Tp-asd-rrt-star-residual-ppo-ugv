# Safety-bounded residual PPO infrastructure

This package is the first testable reinforcement-learning layer for the
terrain/radiation navigation stack. It does **not** replace, tune, or bypass
Pure Pursuit, the frozen PI controller, or the deterministic Safety Gate. A
1024-step surrogate PPO smoke checkpoint exists as pipeline evidence, but it
is labeled **SMOKE_ONLY_NOT_DEPLOYABLE** and has not been accepted for Gazebo,
robot, safety, or performance deployment.

## Control contract

The intended command chain is:

```text
Pure Pursuit reference -> frozen PI feedback -> baseline command
                                             + bounded policy residual
                                             -> /control/base_cmd
                                             -> existing Safety Gate
                                             -> /cmd_vel
```

Policy actions contain exactly two normalized values in `[-1, 1]`:

1. linear-velocity residual, scaled to at most `0.02 m/s`;
2. angular-velocity residual, scaled to at most `0.10 rad/s`.

The combined pre-gate command is additionally clipped to non-reversing
`[0.0, 0.20] m/s` and `[-0.60, 0.60] rad/s`. These are defense-in-depth limits,
not a replacement for the Safety Gate. A zero action returns the exact
baseline `Command` object. A malformed, non-finite, or inference-failing
action falls back to the unchanged finite baseline. If the baseline itself is
non-finite, the residual layer outputs zero; the downstream gate must remain
active. If a finite baseline is already outside the residual layer's absolute
envelope, no learned correction is applied and the unchanged command is left
for the Safety Gate; this prevents absolute clipping from masquerading as a
large policy residual.

## Observation

`observation.py` defines a fixed, normalized 14-value vector:

1. baseline and actual linear speeds;
2. baseline and actual angular speeds;
3. linear and angular tracking errors;
4. lateral and heading errors;
5. Pure Pursuit curvature and remaining goal distance;
6. radiation dose rate and terrain impedance;
7. baseline saturation and safety-stop flags.

Every continuous value must be finite. Invalid observations are encoded as an
invalid all-zero vector, and `SafeResidualController` preserves the baseline
without calling or trusting the policy output.

## Reward and episode status

`reward.py` makes the reward contract explicit. Positive progress toward the
goal is rewarded. Lateral/heading/speed error, residual effort, residual
changes, radiation, terrain impedance, and saturation are penalized. Goal
arrival terminates with a bonus. Collision, out-of-bounds state, Safety Gate
stop, invalid action, or non-finite transition terminates with a dominant
safety penalty. A time limit is a Gymnasium truncation, not a success.

Reward weights are initial engineering values for infrastructure tests only.
They must be reviewed against recorded PID A/B distributions before any long
training or claimed performance comparison.

## Environments and optional dependencies

`ResidualControlCoreEnv` is a deterministic first-order surrogate. It has no
ROS, Gym, Torch, or SB3 dependency and exists to test contracts, seeding, and
short PPO smoke runs. It is not a validated Husky/Gazebo dynamics model.

`make_gym_env()` provides a Gymnasium wrapper when Gymnasium is installed.
PPO training and checkpoint loading additionally require compatible versions
of Stable-Baselines3 and Torch. These packages are optional, imported lazily,
and never downloaded automatically by the ROS package. The Python 3.8 CPU
environment used for the smoke is fully pinned in
`requirements-ppo-py38.txt`; the PyTorch CPU wheel index is declared there.
The workspace-local `.rl_deps/` target is deliberately not package data and
must not be copied into a ROS installation.

Ubuntu's system `mpl_toolkits` namespace can conflict with the pinned
Matplotlib installed into a `--target` directory. Merely prepending
`.rl_deps/` to `PYTHONPATH` does not exclude `/usr/lib/python3/dist-packages`.
Use a clean virtual environment without system site-packages, or use the
documented `python3 -S` invocation so the two Matplotlib installations cannot
be mixed:

```bash
cd /home/i/terrain_radiation_ws
python3 -m pip install \
  --target /home/i/terrain_radiation_ws/.rl_deps \
  --requirement src/risk_aware_residual_rl/requirements-ppo-py38.txt
```

Dependency installation is an explicit provisioning step, not an action
performed by the training or evaluation commands.

Inspect a configuration without optional dependencies or training:

```bash
source /opt/ros/foxy/setup.bash
source /home/i/terrain_radiation_ws/install/setup.bash
ros2 run risk_aware_residual_rl residual_ppo_train \
  --config /home/i/terrain_radiation_ws/src/risk_aware_residual_rl/config/residual_ppo_default.json \
  --dry-run
```

Once dependencies are deliberately provisioned, run the ROS-free module with
an isolated import path. This example reproduces the 1024-step smoke settings
but writes to `/tmp`, leaving the evidence directory immutable:

```bash
cd /home/i/terrain_radiation_ws
PYTHONPATH=/home/i/terrain_radiation_ws/.rl_deps:/home/i/terrain_radiation_ws/src/risk_aware_residual_rl \
python3 -S -m risk_aware_residual_rl.training \
  --config /home/i/terrain_radiation_ws/src/risk_aware_residual_rl/config/residual_ppo_default.json \
  --seed 31 \
  --total-timesteps 1024 \
  --checkpoint-freq 512 \
  --checkpoint-dir /tmp/residual_ppo_reproduction \
  --device cpu
```

The CLI writes periodic SB3 checkpoints, a final checkpoint, and the exact
JSON training configuration. It does not label a checkpoint safe or deploy it.

The completed pipeline smoke evidence is in
`acceptance_logs/rl_ppo_20260822/smoke_checkpoints/`: it contains 512-step,
1024-step, and final checkpoints plus `training_config.json`. These artifacts
prove only that the pinned CPU training pipeline completed 1024 surrogate
steps. Every checkpoint in that directory is
**SMOKE_ONLY_NOT_DEPLOYABLE**.

`acceptance_logs/rl_ppo_20260822/offline_evaluation_smoke.json` records a
three-episode deterministic CLI smoke of the final checkpoint at seeds 31--33.
It checked 900 finite, bounded actions; all three surrogate episodes reached
the 300-step truncation rather than the goal. The negative returns and zero
terminations are pipeline evidence, not a positive performance result, and the
JSON explicitly keeps `deployable: false`.

That first smoke exposed an environment reachability defect rather than a
reason to extend training blindly. The old surrogate sampled 3--6 m goals for
a 300-step, 0.1 s horizon, while its nominal 0.08 m/s baseline could cover at
most 2.4 m before terrain and first-order lag. Its slowdown also approached
zero at the goal tolerance, so the baseline could approach the goal
asymptotically without terminating. The corrected defaults use 0.8--1.8 m,
400 steps, and the real follower's 0.04 m/s minimum tracking speed. Seeded
zero-residual and speed/heading heuristic regression tests now establish that
the default environment is reachable and that the heuristic finishes faster
with a higher return. The original 1024-step checkpoints were trained on the
defective environment and must not be resumed, compared as equivalent, or
deployed.

Training writes both `training_config.json` and
`training_manifest.json`; the latter records the environment, observation,
action-limit, and reward contracts and always marks the artifact non-deployable.
For an existing checkpoint, never add contract fields by hand. Audit and
atomically finalize it with the real pinned ML interpreter:

```bash
PYTHONPATH=/home/i/terrain_radiation_ws/.rl_deps:/home/i/terrain_radiation_ws/src/risk_aware_residual_rl \
/usr/bin/python3 -S -m risk_aware_residual_rl.artifact_finalize \
  --checkpoint acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/residual_ppo_final.zip \
  --training-config acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/training_config.json \
  --manifest acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/training_manifest.json \
  --evidence acceptance_logs/rl_ppo_20260822/retrain50k_artifact_finalization.json
```

The finalizer loads the checkpoint rather than trusting its filename. It
requires its saved observation/action spaces to exactly match the current Gym
environment, verifies seed, rollout-rounded timestep count, and PPO
hyperparameters against the training config, hashes both inputs, generates the
current contract, and runs `validate_artifact_contract` before emitting audit
evidence. Any mismatch refuses finalization; it is not repaired by inventing
metadata or relabeling the checkpoint.

The completed `config/residual_ppo_budget_cpu.json` 20k run failed acceptance:
on held-out seeds 1000--1099, zero residual reached 100% with mean length
260.57 and return -58.15, while PPO reached 99% with length 250.48 and return
-75.89. Reward decomposition attributes nearly all lost return to worse
heading, lateral, and speed tracking; residual effort/change was negligible
and there were no saturation or safety penalties. Diagnostics further show
the deterministic policy emitted a negative angular action on every step and
opposed the heading error only 21.9% of the time.

The root cause is a collapsed angular policy combined with overcompressed
inputs: heading used a pi-radian scale, goal distance a 10 m scale, and
radiation an 8-unit scale despite much smaller surrogate ranges. The corrected
observation scales are 1 radian, 2 m, 0.5 radiation units, and 50 terrain
units. Left/right mirror tests protect the dynamics and reward from directional
bias. No safety, heading, lateral, or residual cost was removed.

One evidence-driven retraining candidate is
`config/residual_ppo_retrain_cpu.json`: 50k CPU steps, seed 47, 1024-step
rollouts, and entropy coefficient 0.001 to reduce another early constant-action
collapse. Do not resume the 20k checkpoint because its observation contract is
obsolete. After training from scratch, evaluate at least 100 held-out
consecutive seeds and require all of the following:

- at least 95% `goal_reached` and no more than 5% time-limit truncations;
- zero invalid/non-finite/out-of-bounds actions and zero safety terminations;
- success rate no more than two percentage points below the zero-residual
  baseline on the identical seeds;
- lower mean episode length and higher mean return than the zero-residual
  baseline.

Failure of any gate means the checkpoint remains offline-only. Passing these
surrogate gates still does not authorize Gazebo or robot deployment.
Run the 50k candidate once from scratch. If it still fails the same-seed return
or success gates, or again collapses to a one-sign angular policy, stop PPO
extension rather than increasing the timestep budget; the next investigation
should be symmetry-aware sampling/curriculum or policy architecture, not more
steps on the same setup.

Evaluate one explicit checkpoint offline with deterministic prediction (the
default), consecutive episode seeds, and JSON output:

```bash
cd /home/i/terrain_radiation_ws
PYTHONPATH=/home/i/terrain_radiation_ws/.rl_deps:/home/i/terrain_radiation_ws/src/risk_aware_residual_rl \
python3 -S -m risk_aware_residual_rl.evaluation \
  --checkpoint acceptance_logs/rl_ppo_20260822/smoke_checkpoints/residual_ppo_final.zip \
  --episodes 10 \
  --seed 31 \
  --output /tmp/residual_ppo_offline_evaluation.json
```

`residual_ppo_evaluate` is also installed as a console entry point for a clean
virtual environment. By default it runs the checkpoint and zero residual on
identical seeds, aggregates signed reward components, and directly applies the
documented gates. JSON distinguishes `execution_valid` from
`acceptance_passed`; an executed evaluation can be valid while acceptance is
false. Invalid execution returns status 2, a valid but rejected A/B returns 3,
and accepted surrogate A/B returns 0. `--no-zero-baseline` performs execution
checks only and leaves `acceptance_passed` null. Non-finite, malformed, or
out-of-`[-1, 1]` actions are rejected before stepping the environment.
Successful acceptance still sets `deployable` to `false` and carries the
`SMOKE_ONLY_NOT_DEPLOYABLE` label. `--stochastic` is available only for
explicit diagnostic runs.

## ROS adapter and launch contract

`residual_policy_node` implements the adapter. The main experiment and online
radiation launch files expose `enable_residual_rl`, policy/checkpoint and
isolated-worker options, timeout/backoff values, and the two residual bounds.

With the default `enable_residual_rl:=false`, the adapter is not launched and
the follower retains its historical `/control/base_cmd` output. With
`enable_residual_rl:=true`, launch atomically remaps the follower to private
`/control/pid_baseline_cmd` and starts exactly one residual publisher on
`/control/base_cmd`. The existing Safety Gate remains subscribed there and is
still the only component that sends final `/cmd_vel`.

The adapter starts by publishing zero until a baseline exists. A missing,
stale, or non-finite baseline produces zero. Stale/missing metrics, model
timeout, model exception, and invalid policy action fall back to the fresh
baseline. Non-finite auxiliary metrics/dose, follower stop/goal/wait states,
and e-stop clear state and publish zero. The `enable_rl` parameter and
`/control/residual_rl_enable` Bool topic are independent runtime kill switches;
when disabled, a fresh baseline passes through unchanged.

Safe zero-policy smoke insertion is explicit:

```bash
ros2 launch risk_aware_planner_cpp \
  tp_asd_rrt_star_online_radiation.launch.py \
  enable_motion:=true \
  enable_residual_rl:=true \
  residual_policy_type:=zero
```

An SB3 policy is never randomly initialized. `residual_policy_type:=sb3`
requires an explicit checkpoint, manifest, and SHA-256 allowlist entry.
Startup rejects a missing artifact, hash mismatch, or a manifest whose ordered
14-field observation/scaling or normalized two-action contract differs from
the running adapter. The zero policy never creates an inference worker.

For learned inference, the ROS parent imports neither Torch nor SB3. It starts
one explicit process group using the configured interpreter, `-S`, and
`residual_worker_pythonpath`; JSON-lines requests carry a monotonic request id,
14 bounded observations, and exactly two bounded actions. Model loading and a
startup handshake complete before control callbacks begin. A typical isolated
50k-candidate invocation is:

```bash
ros2 launch risk_aware_planner_cpp \
  tp_asd_rrt_star_online_radiation.launch.py \
  enable_residual_rl:=true \
  residual_policy_type:=sb3 \
  residual_checkpoint_path:=/home/i/terrain_radiation_ws/acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/residual_ppo_final.zip \
  residual_checkpoint_manifest_path:=/home/i/terrain_radiation_ws/acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/training_manifest.json \
  residual_checkpoint_sha256_allowlist:=8233e2504909a97844cb3f97c72ab7c7756b1762a997a48168904256c2f1c742 \
  residual_worker_python_executable:=/usr/bin/python3 \
  residual_worker_pythonpath:=/home/i/terrain_radiation_ws/.rl_deps \
  residual_model_timeout_sec:=0.05
```

Every prediction has a hard IPC deadline. Worker timeout/crash, malformed
JSON, wrong request id, non-finite action, or action outside `[-1, 1]` kills
the entire worker process group, uses the fresh PID baseline for that cycle,
and fault-latches RL disabled. There is no automatic recovery during control.
After diagnosis, an operator must explicitly disable then enable
`/control/residual_rl_enable` (or set `enable_rl` false then true); bounded
exponential restart backoff defaults to 0.5 s through 5 s. E-stop also kills
and latches the worker. Node shutdown closes the process group so no orphan is
left. The downstream Safety Gate remains the final command authority.

The zero-policy Gazebo wiring smoke evidence is stored under
`acceptance_logs/pid_rl_adapter_20260822/`. In particular,
`runtime/readiness.log` records ready maps/odometry and a non-empty path,
`runtime/stack.log` records the private PID topic and zero-policy adapter
startup, and `zero_policy_bag/` records the baseline, residual output, Safety
Gate output, control metrics, path, and odometry topics. This evidence validates
the adapter wiring and zero-policy run only; it is not learned-policy safety or
performance approval.

`acceptance_logs/rl_ppo_20260822/retrain50k_worker_smoke.json` records five
real-checkpoint requests through this isolated worker, contract validation,
bounded finite actions, no Torch/SB3 import in the parent, and clean shutdown
without an orphan. It is a non-ROS worker smoke, not Gazebo approval.

Training should next move from the surrogate to recorded/offline transitions
and a resettable Gazebo adapter. Deployment requires independent evaluation on
unseen seeds, all existing emergency-stop tests, and an explicit checkpoint
allowlist. The watchdog and offline surrogate gates do not provide those
approvals.
