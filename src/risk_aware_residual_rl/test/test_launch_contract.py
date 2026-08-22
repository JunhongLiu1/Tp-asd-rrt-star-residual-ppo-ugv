import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument


SOURCE_ROOT = Path(__file__).resolve().parents[2]
PLANNER_LAUNCH_DIR = (
    SOURCE_ROOT / "risk_aware_planner_cpp" / "launch"
)
RESIDUAL_ARGUMENTS = {
    "enable_residual_rl",
    "residual_policy_type",
    "residual_checkpoint_path",
    "residual_checkpoint_manifest_path",
    "residual_checkpoint_sha256_allowlist",
    "residual_worker_python_executable",
    "residual_worker_pythonpath",
    "residual_worker_startup_timeout_sec",
    "residual_worker_backoff_initial_sec",
    "residual_worker_backoff_max_sec",
    "residual_baseline_timeout_sec",
    "residual_metrics_timeout_sec",
    "residual_model_timeout_sec",
    "max_linear_residual",
    "max_angular_residual",
}


def load_launch(path, module_name):
    spec = importlib.util.spec_from_file_location(module_name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def declared_arguments(module):
    description = module.generate_launch_description()
    return {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def test_experiment_launch_declares_safe_residual_contract():
    path = PLANNER_LAUNCH_DIR / "tp_asd_rrt_star_experiment.launch.py"
    module = load_launch(path, "residual_experiment_launch")
    assert RESIDUAL_ARGUMENTS <= declared_arguments(module)

    source = path.read_text(encoding="utf-8")
    assert "condition=IfCondition(enable_residual_rl)" in source
    assert "'/control/pid_baseline_cmd' if" in source
    assert '"output_topic": "/control/base_cmd"' in source
    assert '"input_topic": "/control/base_cmd"' in source


def test_online_radiation_launch_forwards_every_residual_argument():
    path = (
        PLANNER_LAUNCH_DIR /
        "tp_asd_rrt_star_online_radiation.launch.py"
    )
    module = load_launch(path, "residual_online_radiation_launch")
    assert RESIDUAL_ARGUMENTS <= declared_arguments(module)
    source = path.read_text(encoding="utf-8")
    for argument in RESIDUAL_ARGUMENTS:
        assert source.count('"{}"'.format(argument)) >= 2
