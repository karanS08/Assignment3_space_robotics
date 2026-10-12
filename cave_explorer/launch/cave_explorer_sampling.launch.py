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
    strategy_launch_arg = DeclareLaunchArgument(
        'strategy',
        default_value='adaptive',
        description='Sampling strategy: adaptive (GP uncertainty) or random (baseline)'
    )
    num_samples_launch_arg = DeclareLaunchArgument(
        'num_samples',
        default_value='30',
        description='Number of measurements to take before stopping'
    )
    seed_launch_arg = DeclareLaunchArgument(
        'seed',
        default_value='0',
        description='Random seed for measurement noise and the random strategy'
    )

    travel_weight_launch_arg = DeclareLaunchArgument(
        'travel_weight',
        default_value='0.3',
        description='Adaptive strategy: how strongly path distance is penalised (0 = ignore distance)'
    )
    log_directory_launch_arg = DeclareLaunchArgument(
        'log_directory',
        default_value='~/cave_sampling_logs',
        description='Where to save the CSV log of each run'
    )

    # Advanced 6: adaptive scientific sampling.
    # Run this instead of cave_explorer_autonomy.launch.py, since both send navigation goals.
    adaptive_sampling_node = Node(
        package='cave_explorer',
        executable='adaptive_sampling',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'strategy': LaunchConfiguration('strategy'),
                     'num_samples': LaunchConfiguration('num_samples'),
                     'seed': LaunchConfiguration('seed'),
                     'travel_weight': LaunchConfiguration('travel_weight'),
                     'log_directory': LaunchConfiguration('log_directory')}]
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(strategy_launch_arg)
    ld.add_action(num_samples_launch_arg)
    ld.add_action(seed_launch_arg)
    ld.add_action(travel_weight_launch_arg)
    ld.add_action(log_directory_launch_arg)
    ld.add_action(adaptive_sampling_node)

    return ld
