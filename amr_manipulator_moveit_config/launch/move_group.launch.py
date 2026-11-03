import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    desc_share = get_package_share_directory('amr_manipulator_description')
    config_share = get_package_share_directory('amr_manipulator_moveit_config')

    # The robot description is the same xacro as in the simulation. The planning frame is the map:
    # the pose of the robot in it comes from TF, so AMCL (or SLAM) must be running, or the
    # static_map_odom argument below must be true.
    moveit_config = (
        MoveItConfigsBuilder('amr_manipulator', package_name='amr_manipulator_moveit_config')
        .robot_description(
            file_path=os.path.join(desc_share, 'urdf', 'amr_manipulator.urdf.xacro'),
            mappings={'use_sim': 'true'})
        .robot_description_semantic(file_path='config/amr_manipulator.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    static_arg = DeclareLaunchArgument(
        'static_map_odom', default_value='false',
        description="'true' publishes an identity transform map -> odom: for tests without AMCL or SLAM "
                    "(the robot starts at the origin, where the two frames coincide)")
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false', description="'true' opens RViz with the MotionPlanning display")
    scene_arg = DeclareLaunchArgument(
        'scene', default_value='true',
        description="'true' loads the boxes of the warehouse SDF (walls, shelves, benches, columns) "
                    "into the planning scene once move_group is up")

    # Waits for move_group, applies the scene, exits
    scene = Node(
        package='amr_manipulator_moveit_config', executable='scene_from_sdf.py', output='screen',
        condition=IfCondition(LaunchConfiguration('scene')))

    static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('static_map_odom')))

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': True,
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                # Wrapped so that the empty string is accepted as a parameter value
                'capabilities': ParameterValue('', value_type=str),
                'disable_capabilities': ParameterValue('', value_type=str),
                'monitor_dynamics': False,
            },
        ],
        additional_env={'DISPLAY': os.environ.get('DISPLAY', '')})

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', os.path.join(config_share, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([static_arg, rviz_arg, scene_arg, static_map_odom, move_group, scene, rviz])
