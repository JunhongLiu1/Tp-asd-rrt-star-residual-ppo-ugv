from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
    enable_frame_alignment = LaunchConfiguration("enable_frame_alignment")


    frame_alignment_node = Node(
        package="risk_aware_planner_cpp",
        executable="map_odom_alignment_node",
        name="map_odom_alignment_cpp",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_frame": "map",
            "geometry_topic": "/ground_truth/radiation_map",
            "risk_map_topic": "/risk_map",
            "metadata_topic": "/risk_map/metadata",
            "odom_frame": "odom",
            "ground_truth_topic": "/ground_truth/odom",
            "filtered_odom_topic": "/odometry/filtered",
        }],
        condition=IfCondition(enable_frame_alignment),
    )

    radiation_mapper = Node(
        package="risk_aware_planner_cpp",
        executable="radiation_online_mapper_node",
        name="radiation_online_mapper_cpp",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_frame": "map",
            "update_radius_m": 2.5,
            "sigma_m": 0.8,
            "dose_to_risk_gain": 80.0,
            "dose_replan_threshold": 0.5,
            "dose_stop_threshold": 8.0,
            "path_risk_threshold": 70,
            "publish_rate_hz": 2.0,
        }],
    )

    experiment_launch = PathJoinSubstitution([
        FindPackageShare("risk_aware_planner_cpp"),
        "launch",
        "tp_asd_rrt_star_experiment.launch.py",
    ])

    online_experiment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(experiment_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "enable_motion": enable_motion,
            "enable_adaptive_sampling": enable_adaptive_sampling,
            "enable_velocity_pid": enable_velocity_pid,
            "enable_residual_rl": enable_residual_rl,
            "residual_policy_type": residual_policy_type,
            "residual_checkpoint_path": residual_checkpoint_path,
            "residual_checkpoint_manifest_path": (
                residual_checkpoint_manifest_path
            ),
            "residual_checkpoint_sha256_allowlist": (
                residual_checkpoint_sha256_allowlist
            ),
            "residual_worker_python_executable": (
                residual_worker_python_executable
            ),
            "residual_worker_pythonpath": residual_worker_pythonpath,
            "residual_worker_startup_timeout_sec": (
                residual_worker_startup_timeout_sec
            ),
            "residual_worker_backoff_initial_sec": (
                residual_worker_backoff_initial_sec
            ),
            "residual_worker_backoff_max_sec": (
                residual_worker_backoff_max_sec
            ),
            "residual_baseline_timeout_sec": residual_baseline_timeout_sec,
            "residual_metrics_timeout_sec": residual_metrics_timeout_sec,
            "residual_model_timeout_sec": residual_model_timeout_sec,
            "max_linear_residual": max_linear_residual,
            "max_angular_residual": max_angular_residual,
            "linear_pid_kp": linear_pid_kp,
            "linear_pid_ki": linear_pid_ki,
            "linear_pid_kd": linear_pid_kd,
            "angular_pid_kp": angular_pid_kp,
            "angular_pid_ki": angular_pid_ki,
            "angular_pid_kd": angular_pid_kd,
            "metrics_csv": metrics_csv,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "enable_adaptive_sampling",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "enable_velocity_pid",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "enable_residual_rl",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "residual_policy_type",
            default_value="zero",
        ),
        DeclareLaunchArgument(
            "residual_checkpoint_path",
            default_value="",
        ),
        DeclareLaunchArgument(
            "residual_checkpoint_manifest_path",
            default_value="",
        ),
        DeclareLaunchArgument(
            "residual_checkpoint_sha256_allowlist",
            default_value="",
        ),
        DeclareLaunchArgument(
            "residual_worker_python_executable",
            default_value="/usr/bin/python3",
        ),
        DeclareLaunchArgument(
            "residual_worker_pythonpath",
            default_value="",
        ),
        DeclareLaunchArgument(
            "residual_worker_startup_timeout_sec",
            default_value="15.0",
        ),
        DeclareLaunchArgument(
            "residual_worker_backoff_initial_sec",
            default_value="0.5",
        ),
        DeclareLaunchArgument(
            "residual_worker_backoff_max_sec",
            default_value="5.0",
        ),
        DeclareLaunchArgument(
            "residual_baseline_timeout_sec",
            default_value="0.30",
        ),
        DeclareLaunchArgument(
            "residual_metrics_timeout_sec",
            default_value="0.50",
        ),
        DeclareLaunchArgument(
            "residual_model_timeout_sec",
            default_value="0.05",
        ),
        DeclareLaunchArgument(
            "max_linear_residual",
            default_value="0.02",
        ),
        DeclareLaunchArgument(
            "max_angular_residual",
            default_value="0.10",
        ),
        DeclareLaunchArgument(
            "linear_pid_kp",
            default_value="0.80",
        ),
        DeclareLaunchArgument(
            "linear_pid_ki",
            default_value="0.10",
        ),
        DeclareLaunchArgument(
            "linear_pid_kd",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "angular_pid_kp",
            default_value="0.50",
        ),
        DeclareLaunchArgument(
            "angular_pid_ki",
            default_value="0.05",
        ),
        DeclareLaunchArgument(
            "angular_pid_kd",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "metrics_csv",
            default_value="/tmp/tp_asd_rrt_star_metrics.csv",
        ),
        DeclareLaunchArgument(
            "enable_frame_alignment",
            default_value="false",
        ),
        frame_alignment_node,
        radiation_mapper,
        online_experiment,
    ])
