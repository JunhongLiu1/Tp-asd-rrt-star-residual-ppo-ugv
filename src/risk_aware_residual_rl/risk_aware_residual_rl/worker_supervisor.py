"""ROS-free hard-timeout supervisor for isolated learned-policy inference."""

from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import time

from .action import ActionLimits
from .observation import OBSERVATION_FIELDS
from .observation import ObservationConfig


class WorkerFailure(RuntimeError):
    """Base class for fail-closed worker errors."""


class WorkerLatched(WorkerFailure):
    """Require an explicit enable after a worker fault."""


class WorkerBackoff(WorkerFailure):
    """Reject restart attempts until bounded backoff expires."""


def sha256_file(path):
    """Hash an artifact without loading it into memory."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_artifact_contract(
    checkpoint_path,
    manifest_path,
    sha256_allowlist,
    action_limits=ActionLimits(),
    observation_config=ObservationConfig(),
):
    """Reject checkpoints not explicitly approved for the local contract."""
    checkpoint = Path(checkpoint_path).expanduser().resolve()
    manifest_file = Path(manifest_path).expanduser().resolve()
    if not checkpoint.is_file():
        raise ValueError("checkpoint does not exist: " + str(checkpoint))
    if not manifest_file.is_file():
        raise ValueError("checkpoint manifest does not exist: " + str(
            manifest_file))
    allowed = {
        item.strip().lower()
        for item in str(sha256_allowlist).replace(",", " ").split()
        if item.strip()
    }
    if not allowed or any(
        len(item) != 64 or any(c not in "0123456789abcdef" for c in item)
        for item in allowed
    ):
        raise ValueError("checkpoint SHA256 allowlist is empty or malformed")
    actual_hash = sha256_file(checkpoint)
    if actual_hash not in allowed:
        raise ValueError(
            "checkpoint SHA256 is not allowlisted: " + actual_hash)
    with manifest_file.open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    if manifest.get("checkpoint_sha256", "").lower() != actual_hash:
        raise ValueError("manifest checkpoint_sha256 does not match artifact")
    if manifest.get("observation_fields") != list(OBSERVATION_FIELDS):
        raise ValueError("manifest observation field contract mismatch")
    if manifest.get("observation") != asdict(observation_config):
        raise ValueError("manifest observation scale contract mismatch")
    if manifest.get("action_limits") != asdict(action_limits):
        raise ValueError("manifest action limit contract mismatch")
    action_contract = manifest.get("action_contract", {})
    if action_contract.get("shape") != [2]:
        raise ValueError("manifest action shape contract mismatch")
    if action_contract.get("normalized_bounds") != [-1.0, 1.0]:
        raise ValueError("manifest normalized action bounds mismatch")
    return {
        "checkpoint": str(checkpoint),
        "manifest": str(manifest_file),
        "sha256": actual_hash,
    }


class InferenceWorkerSupervisor:
    """Own one worker process and enforce a hard deadline per request."""

    def __init__(
        self,
        command,
        prediction_timeout_sec=0.05,
        startup_timeout_sec=15.0,
        backoff_initial_sec=0.5,
        backoff_max_sec=5.0,
        environment=None,
        clock=time.monotonic,
    ):
        values = (
            prediction_timeout_sec,
            startup_timeout_sec,
            backoff_initial_sec,
            backoff_max_sec,
        )
        if not command:
            raise ValueError("worker command must not be empty")
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError(
                "worker timeouts/backoff must be finite and positive")
        if backoff_max_sec < backoff_initial_sec:
            raise ValueError("worker max backoff must be >= initial backoff")
        self.command = list(command)
        self.prediction_timeout_sec = prediction_timeout_sec
        self.startup_timeout_sec = startup_timeout_sec
        self.backoff_initial_sec = backoff_initial_sec
        self.backoff_max_sec = backoff_max_sec
        self.environment = dict(environment or os.environ)
        self.clock = clock
        self.process = None
        self.response_buffer = b""
        self.next_request_id = 1
        self.failure_count = 0
        self.restart_not_before = 0.0
        self.latched_disabled = False
        self.last_failure = ""

    def _read_message(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while b"\n" not in self.response_buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise WorkerFailure("worker response timeout")
            ready, unused_write, unused_error = select.select(
                [self.process.stdout.fileno()], [], [], remaining)
            del unused_write, unused_error
            if not ready:
                raise WorkerFailure("worker response timeout")
            chunk = os.read(self.process.stdout.fileno(), 4096)
            if not chunk:
                raise WorkerFailure("worker exited before response")
            self.response_buffer += chunk
            if len(self.response_buffer) > 65536:
                raise WorkerFailure("worker response exceeded size limit")
        line, self.response_buffer = self.response_buffer.split(b"\n", 1)
        try:
            return json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exception:
            raise WorkerFailure(
                "worker returned malformed JSON") from exception

    def _terminate(self):
        process = self.process
        self.process = None
        self.response_buffer = b""
        if process is None:
            return
        # Signal the process group even if its leader already exited: a
        # crashed worker must not leave an ML-runtime descendant orphaned.
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        if process.poll() is None:
            try:
                process.wait(timeout=0.25)
            except subprocess.TimeoutExpired:
                pass
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        if process.poll() is None:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
        if process.stdin:
            process.stdin.close()
        if process.stdout:
            process.stdout.close()
        if process.stderr:
            process.stderr.close()

    def _record_failure(self, reason):
        self._terminate()
        self.failure_count += 1
        delay = min(
            self.backoff_max_sec,
            self.backoff_initial_sec * (2 ** (self.failure_count - 1)),
        )
        self.restart_not_before = self.clock() + delay
        self.latched_disabled = True
        self.last_failure = reason

    def _start(self):
        if self.clock() < self.restart_not_before:
            raise WorkerBackoff("worker restart backoff is active")
        self.process = subprocess.Popen(
            self.command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            # Never let an undrained ML-runtime diagnostic pipe deadlock the
            # worker. Protocol output is exclusively stdout.
            stderr=subprocess.DEVNULL,
            env=self.environment,
            bufsize=0,
            start_new_session=True,
        )
        try:
            ready = self._read_message(self.startup_timeout_sec)
            if ready != {"protocol": 1, "type": "ready"}:
                raise WorkerFailure("worker startup handshake mismatch")
        except WorkerFailure as exception:
            self._record_failure(str(exception))
            raise

    def start(self):
        """Start and validate the worker before it enters a control cycle."""
        if self.latched_disabled:
            raise WorkerLatched("worker is fault-latched disabled")
        if self.process is not None and self.process.poll() is None:
            return
        if self.process is not None:
            self._record_failure("worker crashed before start")
            raise WorkerFailure("worker crashed before start")
        self._start()

    def enable(self):
        """Explicitly clear the fault latch; backoff still applies."""
        self.latched_disabled = False

    def disable(self):
        """Latch disabled and stop any live worker immediately."""
        self.latched_disabled = True
        self._terminate()

    def predict(self, observation, deterministic=True):
        """Request one action or kill/latch the worker on any violation."""
        if self.latched_disabled:
            raise WorkerLatched("worker is fault-latched disabled")
        values = tuple(observation)
        if len(values) != len(OBSERVATION_FIELDS):
            raise WorkerFailure("observation must contain 14 values")
        values = tuple(float(value) for value in values)
        if not all(math.isfinite(value) and -1.0 <= value <= 1.0
                   for value in values):
            raise WorkerFailure("observation is non-finite or out of bounds")
        if self.process is not None and self.process.poll() is not None:
            self._record_failure("worker crashed between requests")
            raise WorkerFailure("worker crashed between requests")
        if self.process is None:
            try:
                self._start()
            except WorkerBackoff:
                raise
            except WorkerFailure:
                raise
        request_id = self.next_request_id
        self.next_request_id += 1
        request = json.dumps({
            "deterministic": bool(deterministic),
            "id": request_id,
            "observation": values,
        }, separators=(",", ":")).encode("utf-8") + b"\n"
        try:
            self.process.stdin.write(request)
            self.process.stdin.flush()
            response = self._read_message(self.prediction_timeout_sec)
            if (
                not isinstance(response, dict) or
                response.get("id") != request_id
            ):
                raise WorkerFailure("worker response request id mismatch")
            action = response.get("action")
            if not isinstance(action, list) or len(action) != 2:
                raise WorkerFailure("worker action shape mismatch")
            action = (float(action[0]), float(action[1]))
            if not all(math.isfinite(value) for value in action):
                raise WorkerFailure("worker action is non-finite")
            if any(abs(value) > 1.0 for value in action):
                raise WorkerFailure(
                    "worker action is outside normalized bounds")
            return action
        except (BrokenPipeError, OSError, TypeError, ValueError,
                WorkerFailure) as exception:
            self._record_failure(str(exception))
            if isinstance(exception, WorkerFailure):
                raise
            raise WorkerFailure("worker IPC failure") from exception

    def close(self):
        """Idempotently terminate the owned process group."""
        self.latched_disabled = True
        self._terminate()


class WorkerResidualPolicy:
    """ResidualPolicy facade backed only by the stdlib supervisor."""

    def __init__(self, supervisor):
        self.supervisor = supervisor

    @property
    def latched_disabled(self):
        return self.supervisor.latched_disabled

    def predict(self, observation, deterministic=True):
        return self.supervisor.predict(observation, deterministic)

    def start(self):
        self.supervisor.start()

    def enable(self):
        self.supervisor.enable()

    def disable(self):
        self.supervisor.disable()

    def close(self):
        self.supervisor.close()
