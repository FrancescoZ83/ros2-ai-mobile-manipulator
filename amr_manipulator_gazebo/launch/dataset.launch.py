import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gz_share = get_package_share_directory('amr_manipulator_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_arg = DeclareLaunchArgument('world', default_value='warehouse.sdf', description='World file in amr_manipulator_gazebo/worlds')
    world_file = PathJoinSubstitution([gz_share, 'worlds', LaunchConfiguration('world')])

    # The world and a free camera with the sensors of the wrist camera: no robot, no controllers, no Nav2, no MoveIt.
    # The camera is moved by the set_pose service of the world (scripts/stocker.py). Same topics as the robot.
    set_resource_path = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.join(gz_share, 'models'))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s --headless-rendering -v 3 ', world_file]}.items())
    camera = Node(package='ros_gz_sim', executable='create', output='screen',
                  arguments=['-file', os.path.join(gz_share, 'models', 'dataset_camera', 'model.sdf'), '-name', 'dataset_camera'])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': os.path.join(gz_share, 'config', 'bridge.yaml')}])
    return LaunchDescription([world_arg, set_resource_path, gazebo, camera, bridge])
