import os
from glob import glob
from setuptools import setup

package_name = 'radiation_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.json')
        ),
        (
            os.path.join(
                'share',
                package_name,
                'dem',
                'processed'
            ),
            glob('dem/processed/*.npz')
            + glob('dem/processed/*.json')
        ),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join(
                'share',
                package_name,
                'worlds'
            ),
            glob('worlds/*.world')
        ),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='i',
    maintainer_email='i@todo.todo',
    description='Radiation mapping module for terrain-radiation coupled navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
'console_scripts': [
            'ground_truth_execution_path_recorder = radiation_mapping.ground_truth_execution_path_recorder:main',
            'final_asd_rrt_star_planner = radiation_mapping.final_asd_rrt_star_planner:main',
            'aco_planner = radiation_mapping.aco_planner:main',
            'aco_vehicle_safe_planner = radiation_mapping.aco_vehicle_safe_planner:main',
            'aco_trackable_safe_planner = radiation_mapping.aco_trackable_safe_planner:main',
            'terrain_path_evaluator = radiation_mapping.terrain_path_evaluator:main',
            'terrain_layer_publisher = radiation_mapping.terrain_layer_publisher:main',
            'terrain_3d_impedance_map_node = radiation_mapping.terrain_3d_impedance_map_node:main',
            'husky_vehicle_aware_terrain_node = radiation_mapping.husky_vehicle_aware_terrain_node:main',
            'terrain_query_server = radiation_mapping.terrain_query_server:main',
    'radiation_map_node = radiation_mapping.radiation_map_node:main',
    'radiation_query_service = radiation_mapping.radiation_query_service:main',
    'radiation_cost_service = radiation_mapping.radiation_cost_service:main',
    'path_radiation_cost_service = radiation_mapping.path_radiation_cost_service:main',
    'terrain_cost_service = radiation_mapping.terrain_cost_service:main',
    'fusion_cost_service = radiation_mapping.fusion_cost_service:main',
    'path_score_service = radiation_mapping.path_score_service:main',
    'terrain_cost_map_node = radiation_mapping.terrain_cost_map_node:main',
    'fusion_cost_map_node = radiation_mapping.fusion_cost_map_node:main',
    'path_fusion_map_cost_service = radiation_mapping.path_fusion_map_cost_service:main',
    'path_visualization_marker_node = radiation_mapping.path_visualization_marker_node:main',
    'time_aware_path_cost_service = radiation_mapping.time_aware_path_cost_service:main',
    'best_time_aware_path_selector_service = radiation_mapping.best_time_aware_path_selector_service:main',
    'rrt_star_baseline_planner = radiation_mapping.rrt_star_baseline_planner:main',
    'time_aware_rrt_star_planner = radiation_mapping.time_aware_rrt_star_planner:main',
    'apf_time_aware_rrt_star_planner = radiation_mapping.apf_time_aware_rrt_star_planner:main',
    'asd_time_aware_rrt_star_planner = radiation_mapping.asd_time_aware_rrt_star_planner:main',
    'robot_radiation_dose_monitor = radiation_mapping.robot_radiation_dose_monitor:main',
    'asd_path_waypoint_follower = radiation_mapping.asd_path_waypoint_follower:main',
            'formal_path_waypoint_follower = radiation_mapping.formal_path_waypoint_follower:main',
    'experiment_result_analyzer = radiation_mapping.experiment_result_analyzer:main',
    'rviz_asd_time_aware_rrt_star_planner = radiation_mapping.rviz_asd_time_aware_rrt_star_planner:main',
    'rviz_dynamic_path_follower = radiation_mapping.rviz_dynamic_path_follower:main',
    'planner_comparison_recorder = radiation_mapping.planner_comparison_recorder:main',
    'week4_5_algorithm_result_exporter = radiation_mapping.week4_5_algorithm_result_exporter:main',
   
],
},
)
