from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    robot_radius_launch_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.5',
        description='Minimum wall clearance for a cell to count as accessible [m]'
    )

    # Advanced 2: cave geometry analysis
    cave_geometry_node = Node(
        package='cave_explorer',
        executable='cave_geometry',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'robot_radius': LaunchConfiguration('robot_radius')}]
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(robot_radius_launch_arg)
    ld.add_action(cave_geometry_node)

    return ld
