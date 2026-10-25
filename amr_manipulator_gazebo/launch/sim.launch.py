import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    desc_share = get_package_share_directory('amr_manipulator_description')
    gz_share = get_package_share_directory('amr_manipulator_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(gz_share, 'worlds', 'empty_world.sdf')
    xacro_file = os.path.join(desc_share, 'urdf', 'amr_manipulator.urdf.xacro')

    # 1. Resource path: parent dir of the package share dir lets Gazebo resolve
    #    package://<pkg>/... mesh URIs; models/ is for model:// URIs.
    resource_paths = [
        os.path.dirname(desc_share),
        os.path.join(gz_share, 'models'),
    ]
    existing = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if existing:
        resource_paths.append(existing)
    set_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', os.pathsep.join(resource_paths))

    # 2. Gazebo server + GUI. -r starts the simulation running, -v 3 sets log verbosity.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 3 {world_file}'}.items(),
    )

    # 3. robot_state_publisher: runs xacro at launch time and publishes
    #    /robot_description and /tf_static.
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' use_sim:=true']), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 4. Spawn the robot in Gazebo, reading the URDF from the /robot_description topic.
    #    z is a small drop height so the robot settles on its wheels.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'amr_manipulator', '-z', '0.05'],
        output='screen',
    )

    # 5. Bridge: all Gazebo <-> ROS topics are listed in config/bridge.yaml
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(gz_share, 'config', 'bridge.yaml')}],
        output='screen',
    )

    # 6. Load and activate the joint_state_broadcaster once the robot has been spawned.
    #    The controller_manager only exists after the Gazebo plugin loads, so the spawner
    #    waits for it.
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster]))

    arm_controller_spawner = Node(
       package='controller_manager',
        executable='spawner',
        arguments=['arm_model_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
    )

    start_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        set_resource_path,
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        start_jsb_after_spawn,
        start_arm_after_jsb
    ])
