import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gz_share = get_package_share_directory('amr_manipulator_gazebo')
    slam_share = get_package_share_directory('slam_toolbox')

    # The simulation is a separate launch (sim.launch.py): SLAM can be restarted without
    # restarting Gazebo. slam_toolbox's own launch file starts the node with our parameters.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(gz_share, 'config', 'slam_toolbox.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )
    return LaunchDescription([slam])
