
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('rvr')
    default_model_path = os.path.join(pkg_share_dir, 'drvr_description.urdf')
    map_yaml_file = os.path.join(pkg_share_dir, 'map.yaml')
    nav_config_file = os.path.join(pkg_share_dir, 'nav.yaml')
    ekf_config_file = os.path.join(pkg_share_dir, "ekf.yaml")
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file')

    rvr = Node(
        package="rvr",
        executable="rvr",
        name='rvr',
        output="screen"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        output="screen"
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output="screen"
    )

    ekf_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_odom",
        parameters=[ekf_config_file],
        output="screen"
    )

    ekf_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_map",
        parameters=[ekf_config_file],
        output="screen"
    )

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False'}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            "use_composition": "False",
            "use_sim_time": 'False'}.items())

    ld = LaunchDescription()
    ld.add_action(model_arg)
    ld.add_action(rvr)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(ekf_odom)
    ld.add_action(ekf_map)

    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    # ld.add_action(demo_cmd)
    return ld