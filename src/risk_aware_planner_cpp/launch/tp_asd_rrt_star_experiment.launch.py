from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory(
        "risk_aware_planner_cpp"
    )

    config_file = os.path.join(
        package_share,
        "config",
        "tp_asd_rrt_star_online.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_motion = LaunchConfiguration("enable_motion")
    enable_adaptive_sampling = LaunchConfiguration(
        "enable_adaptive_sampling")
    enable_velocity_pid = LaunchConfiguration("enable_velocity_pid")
    enable_residual_rl = LaunchConfiguration("enable_residual_rl")
    residual_policy_type = LaunchConfiguration("residual_policy_type")
    residual_checkpoint_path = LaunchConfiguration(
        "residual_checkpoint_path")
    residual_checkpoint_manifest_path = LaunchConfiguration(
        "residual_checkpoint_manifest_path")
    residual_checkpoint_sha256_allowlist = LaunchConfiguration(
        "residual_checkpoint_sha256_allowlist")
    residual_worker_python_executable = LaunchConfiguration(
        "residual_worker_python_executable")
    residual_worker_pythonpath = LaunchConfiguration(
        "residual_worker_pythonpath")
    residual_worker_startup_timeout_sec = LaunchConfiguration(
        "residual_worker_startup_timeout_sec")
    residual_worker_backoff_initial_sec = LaunchConfiguration(
        "residual_worker_backoff_initial_sec")
    residual_worker_backoff_max_sec = LaunchConfiguration(
        "residual_worker_backoff_max_sec")
    residual_baseline_timeout_sec = LaunchConfiguration(
        "residual_baseline_timeout_sec")
    residual_metrics_timeout_sec = LaunchConfiguration(
        "residual_metrics_timeout_sec")
    residual_model_timeout_sec = LaunchConfiguration(
        "residual_model_timeout_sec")
    max_linear_residual = LaunchConfiguration("max_linear_residual")
    max_angular_residual = LaunchConfiguration("max_angular_residual")
    linear_pid_kp = LaunchConfiguration("linear_pid_kp")
    linear_pid_ki = LaunchConfiguration("linear_pid_ki")
    linear_pid_kd = LaunchConfiguration("linear_pid_kd")
    angular_pid_kp = LaunchConfiguration("angular_pid_kp")
    angular_pid_ki = LaunchConfiguration("angular_pid_ki")
    angular_pid_kd = LaunchConfiguration("angular_pid_kd")
    metrics_csv = LaunchConfiguration("metrics_csv")
    follower_command_topic = PythonExpression([
        "'/control/pid_baseline_cmd' if '",
        enable_residual_rl,
        "'.lower() in ('true', '1', 'yes') else '/control/base_cmd'",
    ])

    planner = Node(
        package="risk_aware_planner_cpp",
        executable="planner_node",
        name="tp_asd_rrt_star_planner_cpp",
        output="screen",
        parameters=[
            config_file,
            {
                "use_sim_time": use_sim_time,
                "enable_adaptive_sampling": ParameterValue(
                    enable_adaptive_sampling, value_type=bool
                ),
            }
        ]
    )

    follower = Node(
        package="risk_aware_planner_cpp",
        executable="path_follower_node",
        name="tp_asd_rrt_star_path_follower_cpp",
        namespace="/",
        output="screen",
        parameters=[
            config_file,
            {
                "use_sim_time": use_sim_time,
                "enable_motion": enable_motion,
                "cmd_vel_topic": ParameterValue(
                    follower_command_topic, value_type=str
                ),
                "enable_velocity_pid": ParameterValue(
                    enable_velocity_pid, value_type=bool
                ),
                "linear_pid.kp": ParameterValue(
                    linear_pid_kp, value_type=float
                ),
                "linear_pid.ki": ParameterValue(
                    linear_pid_ki, value_type=float
                ),
                "linear_pid.kd": ParameterValue(
                    linear_pid_kd, value_type=float
                ),
                "angular_pid.kp": ParameterValue(
                    angular_pid_kp, value_type=float
                ),
                "angular_pid.ki": ParameterValue(
                    angular_pid_ki, value_type=float
                ),
                "angular_pid.kd": ParameterValue(
                    angular_pid_kd, value_type=float
                ),
            },
        ]
    )

    residual_policy = Node(
        package="risk_aware_residual_rl",
        executable="residual_policy_node",
        name="risk_aware_residual_policy",
        namespace="/",
        output="screen",
        condition=IfCondition(enable_residual_rl),
        parameters=[{
            "use_sim_time": use_sim_time,
            "enable_rl": ParameterValue(
                enable_residual_rl, value_type=bool
            ),
            "policy_type": ParameterValue(
                residual_policy_type, value_type=str
            ),
            "checkpoint_path": ParameterValue(
                residual_checkpoint_path, value_type=str
            ),
            "checkpoint_manifest_path": ParameterValue(
                residual_checkpoint_manifest_path, value_type=str
            ),
            "checkpoint_sha256_allowlist": ParameterValue(
                residual_checkpoint_sha256_allowlist, value_type=str
            ),
            "worker_python_executable": ParameterValue(
                residual_worker_python_executable, value_type=str
            ),
            "worker_pythonpath": ParameterValue(
                residual_worker_pythonpath, value_type=str
            ),
            "worker_startup_timeout_sec": ParameterValue(
                residual_worker_startup_timeout_sec, value_type=float
            ),
            "worker_backoff_initial_sec": ParameterValue(
                residual_worker_backoff_initial_sec, value_type=float
            ),
            "worker_backoff_max_sec": ParameterValue(
                residual_worker_backoff_max_sec, value_type=float
            ),
            "baseline_topic": "/control/pid_baseline_cmd",
            "output_topic": "/control/base_cmd",
            "metrics_topic": "/control/pure_pursuit_metrics",
            "follower_status_topic": (
                "/tp_asd_rrt_star_cpp_follower_status"
            ),
            "e_stop_topic": "/e_stop",
            "kill_switch_topic": "/control/residual_rl_enable",
            "baseline_timeout_sec": ParameterValue(
                residual_baseline_timeout_sec, value_type=float
            ),
            "metrics_timeout_sec": ParameterValue(
                residual_metrics_timeout_sec, value_type=float
            ),
            "model_timeout_sec": ParameterValue(
                residual_model_timeout_sec, value_type=float
            ),
            "max_linear_residual": ParameterValue(
                max_linear_residual, value_type=float
            ),
            "max_angular_residual": ParameterValue(
                max_angular_residual, value_type=float
            ),
        }],
    )
    # Foxy/Fast DDS can deadlock endpoint discovery when the Python publisher
    # and C++ Safety Gate subscriber are constructed concurrently.  Let the
    # fail-closed Safety Gate establish its subscriber first.
    residual_policy_start = TimerAction(
        period=1.0,
        actions=[residual_policy],
    )

    safety = Node(
        package="risk_aware_planner_cpp",
        executable="cmd_vel_safety_node",
        name="tp_asd_rrt_star_cmd_vel_safety_cpp",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "enable_motion": enable_motion,
                "allow_reverse": False,
                "input_topic": "/control/base_cmd",
                "output_topic": "/cmd_vel",
                "estop_topic": "/e_stop",
                "max_linear_speed": 0.20,
                "max_angular_speed": 0.60,
                "max_linear_accel": 0.30,
                "max_angular_accel": 1.50,
                "command_timeout_sec": 0.50,
                "publish_rate_hz": 20.0
            }
        ]
    )

    metrics = Node(
        package="risk_aware_planner_cpp",
        executable="planner_metrics_node",
        name="tp_asd_rrt_star_metrics_cpp",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "path_topic": "/tp_asd_rrt_star_cpp_path",
                "status_topic": "/tp_asd_rrt_star_cpp_status",
                "metrics_topic": "/tp_asd_rrt_star_cpp_metrics",
                "radiation_topic": "/risk_map"
            }
        ]
    )

    recorder = Node(
        package="risk_aware_planner_cpp",
        executable="planner_metrics_recorder_node",
        name="tp_asd_rrt_star_metrics_recorder_cpp",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "csv_path": metrics_csv,
                "append": False,
                "flush_every_n": 1
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false"
        ),
        DeclareLaunchArgument(
            "enable_adaptive_sampling",
            default_value="true"
        ),
        DeclareLaunchArgument(
            "enable_velocity_pid",
            default_value="true"
        ),
        DeclareLaunchArgument(
            "enable_residual_rl",
            default_value="false"
        ),
        DeclareLaunchArgument(
            "residual_policy_type",
            default_value="zero"
        ),
        DeclareLaunchArgument(
            "residual_checkpoint_path",
            default_value=""
        ),
        DeclareLaunchArgument(
            "residual_checkpoint_manifest_path",
            default_value=""
        ),
        DeclareLaunchArgument(
            "residual_checkpoint_sha256_allowlist",
            default_value=""
        ),
        DeclareLaunchArgument(
            "residual_worker_python_executable",
            default_value="/usr/bin/python3"
        ),
        DeclareLaunchArgument(
            "residual_worker_pythonpath",
            default_value=""
        ),
        DeclareLaunchArgument(
            "residual_worker_startup_timeout_sec",
            default_value="15.0"
        ),
        DeclareLaunchArgument(
            "residual_worker_backoff_initial_sec",
            default_value="0.5"
        ),
        DeclareLaunchArgument(
            "residual_worker_backoff_max_sec",
            default_value="5.0"
        ),
        DeclareLaunchArgument(
            "residual_baseline_timeout_sec",
            default_value="0.30"
        ),
        DeclareLaunchArgument(
            "residual_metrics_timeout_sec",
            default_value="0.50"
        ),
        DeclareLaunchArgument(
            "residual_model_timeout_sec",
            default_value="0.05"
        ),
        DeclareLaunchArgument(
            "max_linear_residual",
            default_value="0.02"
        ),
        DeclareLaunchArgument(
            "max_angular_residual",
            default_value="0.10"
        ),
        DeclareLaunchArgument(
            "linear_pid_kp",
            default_value="0.80"
        ),
        DeclareLaunchArgument(
            "linear_pid_ki",
            default_value="0.10"
        ),
        DeclareLaunchArgument(
            "linear_pid_kd",
            default_value="0.0"
        ),
        DeclareLaunchArgument(
            "angular_pid_kp",
            default_value="0.50"
        ),
        DeclareLaunchArgument(
            "angular_pid_ki",
            default_value="0.05"
        ),
        DeclareLaunchArgument(
            "angular_pid_kd",
            default_value="0.0"
        ),
        DeclareLaunchArgument(
            "metrics_csv",
            default_value="/tmp/tp_asd_rrt_star_metrics.csv"
        ),
        planner,
        follower,
        safety,
        residual_policy_start,
        metrics,
        recorder
    ])
