"""ROS 2 adapter that inserts a bounded residual before the Safety Gate."""

import math
import os
from pathlib import Path
import re
import time

from geometry_msgs.msg import Twist
from radiation_interfaces.msg import ControlMetrics
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String

from .action import ActionLimits
from .action import Command
from .adapter_core import AdapterConfig
from .adapter_core import AdapterInputs
from .adapter_core import ResidualAdapterStateMachine
from .observation import Observation
from .policy import SafeResidualController
from .policy import load_policy
from .worker_supervisor import InferenceWorkerSupervisor
from .worker_supervisor import validate_artifact_contract
from .worker_supervisor import WorkerResidualPolicy


STOP_STATUS_PREFIXES = (
    "GOAL_REACHED",
    "STOPPED",
    "E_STOP_ACTIVE",
    "DISABLED",
    "WAITING_PATH",
    "WAITING_ODOM",
    "INVALID_PATH",
)


class ResidualPolicyNode(Node):
    """Safely combine private PID baseline commands with policy residuals."""

    def __init__(self):
        super().__init__("risk_aware_residual_policy")
        self.declare_parameter("enable_rl", False)
        self.declare_parameter("policy_type", "zero")
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("checkpoint_manifest_path", "")
        self.declare_parameter("checkpoint_sha256_allowlist", "")
        self.declare_parameter("worker_python_executable", "/usr/bin/python3")
        self.declare_parameter("worker_pythonpath", "")
        self.declare_parameter("worker_startup_timeout_sec", 15.0)
        self.declare_parameter("worker_backoff_initial_sec", 0.5)
        self.declare_parameter("worker_backoff_max_sec", 5.0)
        self.declare_parameter(
            "baseline_topic", "/control/pid_baseline_cmd")
        self.declare_parameter("output_topic", "/control/base_cmd")
        self.declare_parameter(
            "metrics_topic", "/control/pure_pursuit_metrics")
        self.declare_parameter(
            "follower_status_topic",
            "/tp_asd_rrt_star_cpp_follower_status")
        self.declare_parameter("e_stop_topic", "/e_stop")
        self.declare_parameter(
            "kill_switch_topic", "/control/residual_rl_enable")
        self.declare_parameter(
            "dose_topic", "/radiation/dose_rate_usv_h")
        self.declare_parameter(
            "status_topic", "/control/residual_rl_status")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("baseline_timeout_sec", 0.30)
        self.declare_parameter("metrics_timeout_sec", 0.50)
        self.declare_parameter("model_timeout_sec", 0.05)
        self.declare_parameter("max_linear_residual", 0.02)
        self.declare_parameter("max_angular_residual", 0.10)
        self.declare_parameter("min_linear_command", 0.0)
        self.declare_parameter("max_linear_command", 0.20)
        self.declare_parameter("max_angular_command", 0.60)

        self.enable_rl = bool(self.get_parameter("enable_rl").value)
        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value)
        if (
            not math.isfinite(self.publish_rate_hz) or
            self.publish_rate_hz <= 0
        ):
            raise ValueError("publish_rate_hz must be finite and positive")

        self.adapter = ResidualAdapterStateMachine(AdapterConfig(
            baseline_timeout_sec=float(
                self.get_parameter("baseline_timeout_sec").value),
            metrics_timeout_sec=float(
                self.get_parameter("metrics_timeout_sec").value),
            model_timeout_sec=float(
                self.get_parameter("model_timeout_sec").value),
        ))
        action_limits = ActionLimits(
            max_linear_residual=float(
                self.get_parameter("max_linear_residual").value),
            max_angular_residual=float(
                self.get_parameter("max_angular_residual").value),
            min_linear_command=float(
                self.get_parameter("min_linear_command").value),
            max_linear_command=float(
                self.get_parameter("max_linear_command").value),
            max_angular_command=float(
                self.get_parameter("max_angular_command").value),
        )
        self.policy = self._load_policy(action_limits)
        self.controller = SafeResidualController(
            self.policy, action_limits=action_limits, deterministic=True)

        self.baseline = None
        self.baseline_received_sec = None
        self.baseline_sequence = 0
        self.metrics = None
        self.metrics_received_sec = None
        self.auxiliary_inputs_finite = True
        self.dose_rate = 0.0
        self.goal_distance = 0.0
        self.emergency_stop = False
        self.motion_stopped = False
        self.last_output = Command(0.0, 0.0)
        self.last_output_baseline_sequence = -1
        self.last_status = ""

        baseline_topic = str(self.get_parameter("baseline_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        if baseline_topic == output_topic:
            raise ValueError("baseline_topic and output_topic must differ")
        self.command_publisher = self.create_publisher(Twist, output_topic, 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), status_qos)

        self.create_subscription(
            Twist, baseline_topic, self._baseline_callback, 10)
        self.create_subscription(
            ControlMetrics,
            str(self.get_parameter("metrics_topic").value),
            self._metrics_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("follower_status_topic").value),
            self._follower_status_callback,
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("dose_topic").value),
            self._dose_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("e_stop_topic").value),
            self._e_stop_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("kill_switch_topic").value),
            self._kill_switch_callback,
            10,
        )
        self.add_on_set_parameters_callback(self._parameter_callback)
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._timer_callback)

        self._publish_command(Command(0.0, 0.0))
        self._publish_status("WAITING_BASELINE: zero command")
        startup_message = (
            "Residual adapter started: policy={}, enable_rl={}, {} -> {}"
        )
        self.get_logger().info(
            startup_message.format(
                str(self.get_parameter("policy_type").value),
                str(self.enable_rl),
                baseline_topic,
                output_topic,
            )
        )

    def _load_policy(self, action_limits):
        policy_type = str(
            self.get_parameter("policy_type").value).strip().lower()
        checkpoint = str(
            self.get_parameter("checkpoint_path").value).strip()
        if policy_type == "zero":
            return load_policy("zero", "")
        if policy_type != "sb3":
            raise ValueError("policy_type must be 'zero' or 'sb3'")
        manifest = str(
            self.get_parameter("checkpoint_manifest_path").value).strip()
        allowlist = str(
            self.get_parameter("checkpoint_sha256_allowlist").value).strip()
        artifact = validate_artifact_contract(
            checkpoint,
            manifest,
            allowlist,
            action_limits=action_limits,
        )
        executable = str(
            self.get_parameter("worker_python_executable").value).strip()
        if not executable:
            raise ValueError("worker_python_executable must not be empty")
        configured_pythonpath = str(
            self.get_parameter("worker_pythonpath").value).strip()
        package_root = str(Path(__file__).resolve().parents[1])
        pythonpath_parts = [
            value for value in (configured_pythonpath, package_root) if value
        ]
        worker_environment = dict(os.environ)
        worker_environment["PYTHONPATH"] = os.pathsep.join(pythonpath_parts)
        command = [
            executable,
            "-S",
            "-u",
            "-m",
            "risk_aware_residual_rl.policy_worker",
            "--checkpoint",
            artifact["checkpoint"],
        ]
        supervisor = InferenceWorkerSupervisor(
            command,
            prediction_timeout_sec=float(
                self.get_parameter("model_timeout_sec").value),
            startup_timeout_sec=float(
                self.get_parameter("worker_startup_timeout_sec").value),
            backoff_initial_sec=float(
                self.get_parameter("worker_backoff_initial_sec").value),
            backoff_max_sec=float(
                self.get_parameter("worker_backoff_max_sec").value),
            environment=worker_environment,
        )
        policy = WorkerResidualPolicy(supervisor)
        # Load and handshake before any baseline callback can block on the ML
        # runtime. A startup failure therefore refuses node startup.
        policy.start()
        return policy

    def _set_policy_enabled(self, enabled):
        method_name = "enable" if enabled else "disable"
        method = getattr(self.policy, method_name, None)
        if callable(method):
            method()

    def _parameter_callback(self, parameters):
        result = SetParametersResult()
        result.successful = True
        for parameter in parameters:
            if parameter.name == "enable_rl":
                self.enable_rl = bool(parameter.value)
                self._set_policy_enabled(self.enable_rl)
                self._reset_policy_state()
                self._publish_for_latest(run_policy=False)
        return result

    @staticmethod
    def _metrics_finite(message):
        values = (
            message.lateral_error_m,
            message.heading_error_rad,
            message.curvature,
            message.reference_linear_mps,
            message.actual_linear_mps,
            message.reference_angular_rps,
            message.actual_angular_rps,
            message.linear_jerk_mps3,
            message.terrain_impedance,
        )
        return all(math.isfinite(value) for value in values)

    def _baseline_callback(self, message):
        now_sec = time.monotonic()
        self.baseline = Command(message.linear.x, message.angular.z)
        self.baseline_received_sec = now_sec
        self.baseline_sequence += 1
        self._publish_for_latest(run_policy=True)

    def _metrics_callback(self, message):
        self.metrics = message
        self.metrics_received_sec = time.monotonic()
        self.auxiliary_inputs_finite = self._metrics_finite(message)
        if not self.auxiliary_inputs_finite:
            self._clear_and_publish_zero("NONFINITE_METRICS")

    def _dose_callback(self, message):
        if not math.isfinite(message.data):
            self.auxiliary_inputs_finite = False
            self._clear_and_publish_zero("NONFINITE_DOSE")
            return
        self.dose_rate = max(0.0, message.data)
        if self.metrics is None or self._metrics_finite(self.metrics):
            self.auxiliary_inputs_finite = True

    def _follower_status_callback(self, message):
        status = message.data.strip()
        goal_match = re.search(r"goal_distance=([^,\s]+)", status)
        if goal_match:
            try:
                value = float(goal_match.group(1))
            except (TypeError, ValueError, OverflowError):
                self.auxiliary_inputs_finite = False
                self._clear_and_publish_zero("INVALID_GOAL_DISTANCE")
                return
            if math.isfinite(value):
                self.goal_distance = max(0.0, value)
            else:
                self.auxiliary_inputs_finite = False
                self._clear_and_publish_zero("NONFINITE_GOAL_DISTANCE")
                return
        if status.startswith(STOP_STATUS_PREFIXES):
            self.motion_stopped = True
            self._clear_and_publish_zero("FOLLOWER_" + status.split(":")[0])
        elif status.startswith(
            ("TRACKING", "ALIGNING_TO_PATH", "PATH_RECEIVED")
        ):
            self.motion_stopped = False

    def _e_stop_callback(self, message):
        self.emergency_stop = bool(message.data)
        if self.emergency_stop:
            self.enable_rl = False
            self._set_policy_enabled(False)
            self._clear_and_publish_zero("E_STOP_ACTIVE")
        else:
            self._reset_policy_state()

    def _kill_switch_callback(self, message):
        self.enable_rl = bool(message.data)
        self._set_policy_enabled(self.enable_rl)
        self._reset_policy_state()
        self._publish_for_latest(run_policy=False)

    def _adapter_inputs(self, now_sec):
        return AdapterInputs(
            now_sec=now_sec,
            baseline=self.baseline,
            baseline_received_sec=self.baseline_received_sec,
            metrics_received_sec=self.metrics_received_sec,
            auxiliary_inputs_finite=self.auxiliary_inputs_finite,
            enable_rl=self.enable_rl,
            emergency_stop=self.emergency_stop,
            motion_stopped=self.motion_stopped,
        )

    def _build_observation(self):
        return Observation(
            baseline_linear=self.baseline.linear,
            actual_linear=self.metrics.actual_linear_mps,
            baseline_angular=self.baseline.angular,
            actual_angular=self.metrics.actual_angular_rps,
            lateral_error=self.metrics.lateral_error_m,
            heading_error=self.metrics.heading_error_rad,
            curvature=self.metrics.curvature,
            goal_distance=self.goal_distance,
            radiation_dose_rate=self.dose_rate,
            terrain_impedance=self.metrics.terrain_impedance,
            baseline_saturated=self.metrics.saturated,
            safety_stop_active=self.emergency_stop,
        )

    def _publish_for_latest(self, run_policy):
        now_sec = time.monotonic()
        decision = self.adapter.evaluate(self._adapter_inputs(now_sec))
        if not decision.apply_policy or not run_policy:
            command = decision.command
            reason = decision.reason
            if decision.apply_policy and not run_policy:
                command = self.last_output
                reason = "holding_last_policy_command"
            if decision.clear_policy_state:
                self._reset_policy_state()
            self._publish_command(command)
            self._publish_status(reason)
            return

        inference_start = time.monotonic()
        policy_decision = self.controller.command(
            self.baseline, self._build_observation())
        inference_duration = time.monotonic() - inference_start
        post_inference = self.adapter.evaluate(
            self._adapter_inputs(time.monotonic()))
        if not post_inference.apply_policy:
            final = post_inference
        else:
            final = self.adapter.finalize_policy(
                self.baseline,
                policy_decision.application,
                inference_duration,
            )
        if final.clear_policy_state:
            self._reset_policy_state()
        self.last_output = final.command
        self.last_output_baseline_sequence = self.baseline_sequence
        self._publish_command(final.command)
        policy_error = policy_decision.policy_error
        status = final.reason
        if policy_error:
            status += ":" + policy_error
        if getattr(self.policy, "latched_disabled", False):
            self.enable_rl = False
            status += ":RL_LATCHED_DISABLED"
        self._publish_status(status)

    def _timer_callback(self):
        self._publish_for_latest(run_policy=False)

    def _reset_policy_state(self):
        reset_method = getattr(self.policy, "reset", None)
        if callable(reset_method):
            reset_method()
        self.last_output = Command(0.0, 0.0)
        self.last_output_baseline_sequence = -1

    def _clear_and_publish_zero(self, reason):
        self.baseline = None
        self.baseline_received_sec = None
        self.metrics = None
        self.metrics_received_sec = None
        self._reset_policy_state()
        self._publish_command(Command(0.0, 0.0))
        self._publish_status(reason + ": zero command")

    def _publish_command(self, command):
        message = Twist()
        message.linear.x = command.linear
        message.angular.z = command.angular
        self.command_publisher.publish(message)

    def _publish_status(self, status):
        if status == self.last_status:
            return
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.last_status = status

    def close(self):
        close_method = getattr(self.policy, "close", None)
        if callable(close_method):
            close_method()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ResidualPolicyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
