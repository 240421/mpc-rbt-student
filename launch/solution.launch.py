import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')
    simulator_launch = os.path.join(get_package_share_directory('mpc_rbt_simulator'), 'launch', 'simulation.launch.py')

    
    return LaunchDescription([
        # Launch RViz2 with configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),

        TimerAction(
            period = 3.0,
            actions = [
                IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulator_launch)
            )
            ] 
        ),

        # Launch localization node (modify node name & parameters as needed)
        TimerAction(
        period = 10.0,
        actions = [
            Node(
                package='mpc_rbt_student',
                executable='localization',
                name='localization',
                output='screen'
            )
        ]
        )

       

    ])
