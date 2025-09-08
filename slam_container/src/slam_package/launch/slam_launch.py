from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Объявление аргументов
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_dir = get_package_share_directory('slam_package')

    params_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    # Узлы и действия
    return LaunchDescription([
        # Публикация статических преобразований tf
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        #    output='screen'
        #),
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        #    output='screen'
        #),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
            output='screen'
        ),

        # Запуск SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file]
        )
    ])
