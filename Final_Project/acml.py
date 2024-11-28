import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('map_file', default_value='$(find your_package)/maps/your_map.yaml', description='Map file'),

        # Start AMCL Node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file': LaunchConfiguration('map_file'),
                # Add additional parameters here (e.g., AMCL config parameters)
            }],
            remappings=[('/scan', '/your_robot/scan')],  # Adjust your scan topic name
        ),
        
        # Optionally log AMCL information
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('use_sim_time', 'true'),
            msg="Using simulation time."
        ),
    ])

Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[{
        'yaml_filename': '$(find your_package)/maps/your_map.yaml'
    }]
)