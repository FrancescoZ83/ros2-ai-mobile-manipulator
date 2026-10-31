import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gz_share = get_package_share_directory('amr_manipulator_gazebo')
    bringup_share = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(gz_share, 'maps', 'warehouse.yaml'),
        description='Full path to the map yaml file')

    # Nav2's own bringup starts map_server, AMCL and the navigation stack (planner, controller,
    # behaviors, BT navigator, velocity smoother) with our parameters. The simulation is a
    # separate launch (sim.launch.py). Do not run it together with slam.launch.py: both
    # would publish the transform map -> odom.
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': os.path.join(gz_share, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )
    return LaunchDescription([map_arg, bringup])
