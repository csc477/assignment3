from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def noise_parameter(name):
    # value_type=float: a command-line "0" would otherwise arrive as an integer
    # and be rejected by the node's double parameter.
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    rviz_config = Path(get_package_share_directory('estimation_assignment')) / 'config' / 'csc477.rviz'
    return LaunchDescription([
        # No defaults: choose the odometry noise explicitly for every run.
        DeclareLaunchArgument('odometry_orientation_noise_std_dev',
                              description='yaw noise added to odometry, in degrees; the mapper applies twice this std. dev.'),
        DeclareLaunchArgument('odometry_position_noise_std_dev',
                              description='std. dev. of the x and y noise added to odometry, in metres'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='also start rviz2 with config/csc477.rviz'),
        Node(package='estimation_assignment',
             executable='occupancy_grid_mapper.py',
             name='occupancy_grid_mapper',
             output='screen',
             parameters=[{
                 'odometry_orientation_noise_std_dev': noise_parameter('odometry_orientation_noise_std_dev'),
                 'odometry_position_noise_std_dev': noise_parameter('odometry_position_noise_std_dev'),
                 'use_sim_time': True,
             }]),
        # Simulation time comes from the /clock recorded in the bag. A rviz2 on
        # wall-clock time would consider every transform in the bag 50 years
        # old and draw nothing in the TF display.
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             output='screen',
             arguments=['-d', str(rviz_config)],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
