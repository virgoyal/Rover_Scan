# ~/vir_ws/src/crack_scan_sim/launch/scan_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_path = os.path.join(
        os.getenv('HOME'),
        'vir_ws/src/crack_scan_sim/config/plate_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='crack_scan_sim',
            executable='rover_position_node',
            name='rover_position_node',
            output='screen',
            parameters=[{'config_path': config_path}]
        ),
        Node(
            package='crack_scan_sim',
            executable='sensor_position_node',
            name='sensor_position_node',
            output='screen',
            parameters=[{'config_path': config_path}]
        ),
        Node(
            package='crack_scan_sim',
            executable='sensor_value_node',
            name='sensor_value_node',
            output='screen'
        ),
        Node(
            package='crack_scan_sim',
            executable='matrix_visualization_node',
            name='matrix_visualization_node',
            parameters=[{
                'width': 100,
                'height': 100,
                'refresh_interval': 20,
            }],
        ),
    ])