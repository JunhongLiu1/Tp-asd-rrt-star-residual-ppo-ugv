import json
from pathlib import Path
import sys

import pytest

from risk_aware_residual_rl.action import ActionLimits
from risk_aware_residual_rl.observation import OBSERVATION_FIELDS
from risk_aware_residual_rl.observation import ObservationConfig
from risk_aware_residual_rl.worker_supervisor import InferenceWorkerSupervisor
from risk_aware_residual_rl.worker_supervisor import WorkerBackoff
from risk_aware_residual_rl.worker_supervisor import WorkerFailure
from risk_aware_residual_rl.worker_supervisor import WorkerLatched
from risk_aware_residual_rl.worker_supervisor import sha256_file
from risk_aware_residual_rl.worker_supervisor import validate_artifact_contract


WORKER_CODE = r"""
import json, os, sys, time
mode_path = sys.argv[1]
mode = open(mode_path).read().strip()
print(json.dumps({'protocol': 1, 'type': 'ready'}), flush=True)
for line in sys.stdin:
    request = json.loads(line)
    if mode == 'hang':
        time.sleep(60)
    elif mode == 'crash':
        os._exit(7)
    elif mode == 'bad_json':
        print('{bad', flush=True)
    elif mode == 'bad_action':
        response = {'id': request['id'], 'action': [2.0, 0.0]}
        print(json.dumps(response), flush=True)
    elif mode == 'nan_action':
        response = {'id': request['id'], 'action': [float('nan'), 0.0]}
        print(json.dumps(response), flush=True)
    elif mode == 'wrong_id':
        response = {'id': request['id'] + 1, 'action': [0.0, 0.0]}
        print(json.dumps(response), flush=True)
    else:
        response = {'id': request['id'], 'action': [0.25, -0.5]}
        print(json.dumps(response), flush=True)
"""


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


def make_supervisor(tmp_path, mode="normal", clock=None, timeout=0.2):
    mode_path = tmp_path / "mode.txt"
    mode_path.write_text(mode, encoding="utf-8")
    supervisor = InferenceWorkerSupervisor(
        [sys.executable, "-u", "-c", WORKER_CODE, str(mode_path)],
        prediction_timeout_sec=timeout,
        startup_timeout_sec=1.0,
        backoff_initial_sec=0.1,
        backoff_max_sec=0.4,
        clock=clock or FakeClock(),
    )
    return supervisor, mode_path


def observation():
    return (0.0,) * len(OBSERVATION_FIELDS)


def test_normal_request_uses_matching_id_and_clean_close(tmp_path):
    supervisor, unused_path = make_supervisor(tmp_path)
    del unused_path
    assert supervisor.predict(observation()) == (0.25, -0.5)
    process = supervisor.process
    assert process.poll() is None
    supervisor.close()
    assert process.poll() is not None
    supervisor.close()


@pytest.mark.parametrize(
    "mode, expected",
    [
        ("crash", "exited"),
        ("bad_json", "malformed JSON"),
        ("bad_action", "outside normalized bounds"),
        ("nan_action", "non-finite"),
        ("wrong_id", "request id mismatch"),
    ],
)
def test_worker_protocol_faults_kill_and_latch(tmp_path, mode, expected):
    supervisor, unused_path = make_supervisor(tmp_path, mode=mode)
    del unused_path
    with pytest.raises(WorkerFailure) as error:
        supervisor.predict(observation())
    assert expected in str(error.value)
    assert supervisor.latched_disabled
    assert supervisor.process is None
    with pytest.raises(WorkerLatched):
        supervisor.predict(observation())
    supervisor.close()


def test_hard_timeout_kills_worker_and_latches(tmp_path):
    supervisor, unused_path = make_supervisor(
        tmp_path, mode="hang", timeout=0.05)
    del unused_path
    with pytest.raises(WorkerFailure) as error:
        supervisor.predict(observation())
    assert "timeout" in str(error.value)
    assert supervisor.process is None
    assert supervisor.latched_disabled


def test_explicit_enable_respects_backoff_then_restarts(tmp_path):
    clock = FakeClock()
    supervisor, mode_path = make_supervisor(
        tmp_path, mode="crash", clock=clock)
    with pytest.raises(WorkerFailure):
        supervisor.predict(observation())
    mode_path.write_text("normal", encoding="utf-8")
    supervisor.enable()
    with pytest.raises(WorkerBackoff):
        supervisor.predict(observation())
    clock.value = 0.1
    assert supervisor.predict(observation()) == (0.25, -0.5)
    supervisor.close()


def test_artifact_allowlist_and_manifest_contract(tmp_path):
    checkpoint = tmp_path / "model.zip"
    checkpoint.write_bytes(b"explicit model bytes")
    digest = sha256_file(checkpoint)
    manifest = {
        "action_contract": {
            "normalized_bounds": [-1.0, 1.0],
            "shape": [2],
        },
        "action_limits": ActionLimits().__dict__,
        "checkpoint_sha256": digest,
        "observation": ObservationConfig().__dict__,
        "observation_fields": list(OBSERVATION_FIELDS),
    }
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    result = validate_artifact_contract(
        checkpoint, manifest_path, digest)
    assert result["sha256"] == digest

    manifest["observation_fields"] = ["wrong"]
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError) as error:
        validate_artifact_contract(checkpoint, manifest_path, digest)
    assert "observation field contract" in str(error.value)


def test_unallowlisted_checkpoint_is_rejected_before_worker_start(tmp_path):
    checkpoint = tmp_path / "model.zip"
    checkpoint.write_bytes(b"model")
    manifest = tmp_path / "manifest.json"
    manifest.write_text("{}", encoding="utf-8")
    with pytest.raises(ValueError) as error:
        validate_artifact_contract(checkpoint, manifest, "0" * 64)
    assert "not allowlisted" in str(error.value)


def test_ros_parent_modules_do_not_import_ml_runtime():
    package = Path(__file__).parents[1] / "risk_aware_residual_rl"
    for filename in ("residual_policy_node.py", "worker_supervisor.py"):
        source = (package / filename).read_text(encoding="utf-8")
        assert "import torch" not in source
        assert "stable_baselines3" not in source
