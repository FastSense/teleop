import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    default_config = os.path.join(
        get_package_share_directory('teleop'),
        'rosbot_gz_sim.yaml'
    )

    config = launch.substitutions.LaunchConfiguration(
        'config',
        default=default_config
    )

    return LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='config file path'
        ),

        # launh Teleop node
        Node(
            package='teleop',
            executable='teleop',
            name='teleop',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        ),

    ])
