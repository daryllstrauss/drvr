import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('rvr')
    rvr_node = launch_ros.actions.Node(
        package="rvr",
        executable="rvr"
    )
    localization_config = os.path.join(
        get_package_share_directory('rvr'),
        'drvr_ukf_config.yaml'
    )
    print("Config", localization_config)
    localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable='ukf_node',
        name="drvr_ukf_node",
        parameters=[localization_config])
    return launch.LaunchDescription([
        rvr_node,
        localization_node
    ])
