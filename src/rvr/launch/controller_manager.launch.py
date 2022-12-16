import launch
import xacro
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os

def generate_launch_description():
    rvr_node = launch_ros.actions.Node(
        package="rvr",
        executable="rvr"
    )
    pkg_share_dir = get_package_share_directory('rvr')
    controllers_file = os.path.join(
        pkg_share_dir,
        'ros2_control_controllers.yaml'
    )
    model_path = os.path.join(pkg_share_dir, 'drvr_description.urdf')
    model_description = xacro.parse(open(model_path))
    xacro.process_doc(model_description)
    robot_description = {'robot_description': model_description.toxml()}

    controller_manager = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file
        ],
        output='both'
    )
    # robot_controller_spawner = launch_ros.actions.Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diffbot_base_controller", "-c", "/controller_manager"],
    # )


    # diff_drive_controller_node = launch_ros.actions.Node(
    #     package="diff_drive_controller",
    #     executable="diff_drive_controller",
    #     arguments=["-c", "/controller_manager"]
    # )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    # joint_state_broadcaster_spawner = launch_ros.actions.Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    return launch.LaunchDescription([
        rvr_node,
        controller_manager,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
