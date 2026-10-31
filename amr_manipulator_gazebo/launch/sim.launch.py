import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('amr_manipulator_description')
    gz_share = get_package_share_directory('amr_manipulator_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # World file name (inside amr_manipulator_gazebo/worlds) and spawn pose of the robot,
    # all chosen from the command line:
    #   ros2 launch amr_manipulator_gazebo sim.launch.py world:=warehouse.sdf x:=-4.0 yaw:=1.57
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.sdf',
        description='World file in amr_manipulator_gazebo/worlds')
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description="'false' runs the Gazebo server only (no GUI), which saves a lot of CPU")

    world_file = PathJoinSubstitution([gz_share, 'worlds', LaunchConfiguration('world')])
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
    #    gz_args is a list of substitutions because the world file name is a launch argument.
    #    Without the GUI the server runs alone (-s) and the sensors render headless.
    gz_flags = PythonExpression([
        "'-r -v 3 ' if '", LaunchConfiguration('gui'),
        "' == 'true' else '-r -s --headless-rendering -v 3 '"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [gz_flags, world_file]}.items(),
    )

    # 3. robot_state_publisher: runs xacro at launch time and publishes
    #    /robot_description and /tf_static. The test payload at the TCP is a launch
    #    argument: ros2 launch amr_manipulator_gazebo sim.launch.py payload_mass:=3.0
    payload_arg = DeclareLaunchArgument(
        'payload_mass',
        default_value='0.0',
        description='Test payload at the TCP [kg], simulation only')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_sim:=true',
            ' payload_mass:=', LaunchConfiguration('payload_mass'),
        ]),
        value_type=str)
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
        arguments=['-topic', 'robot_description', '-name', 'amr_manipulator',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.05', '-Y', LaunchConfiguration('yaw')],
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

    # 7. The arm controller starts once the joint_state_broadcaster spawner has finished.
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_model_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    start_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller_spawner],
        )
    )

    # 8. The gripper controller starts once the arm controller spawner has finished.
    #    On activation it opens the fingers from wherever they ended up during the start-up ramp.
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    start_gripper_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # 9. Stow: once every controller is active, fold the arm into the travel pose
    #    (with the arm folded, the whole-robot footprint equals the platform footprint).
    #    scripts/arm_goto.py waits for the start-up ramp to finish, sends the goal and
    #    prints the tracking error, so the log of the launch is the success test.
    #    The script is found relative to this file: this needs `colcon build --symlink-install`.
    stow_arg = DeclareLaunchArgument(
        'stow', default_value='true',
        description="'false' leaves the arm in the start-up pose (home90) for manipulation tests")
    arm_goto = os.path.normpath(os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '..', '..', 'scripts', 'arm_goto.py'))
    travel_pose = ['3.1416', '-1.5708', '1.5708', '-1.5708', '-1.5708', '1.5708']
    if os.path.isfile(arm_goto):
        stow = ExecuteProcess(
            cmd=['python3', '-u', arm_goto, '--goal', *travel_pose, '--time', '6'],
            condition=IfCondition(LaunchConfiguration('stow')),
            output='screen',
        )
    else:
        stow = LogInfo(msg=f'stow skipped: {arm_goto} not found (build with --symlink-install)')
    stow_after_gripper = RegisterEventHandler(
        OnProcessExit(target_action=gripper_controller_spawner, on_exit=[stow]))

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        yaw_arg,
        gui_arg,
        payload_arg,
        stow_arg,
        set_resource_path,
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        start_jsb_after_spawn,
        start_arm_after_jsb,
        start_gripper_after_arm,
        stow_after_gripper,
    ])
