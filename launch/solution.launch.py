import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    
    return LaunchDescription([
        # Launch localization node (modify node name & parameters as needed)
        Node(
            package='mpc_rbt_student',
            executable='localization',
            name='localization',
            output='screen'
        ),

        # Launch RViz2 with configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )

    ])
