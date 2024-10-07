import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('map_explorer_bringup'),
        'config',
        'params.yaml'
        )
    
    node=Node(
        package = 'map_explorer',
        name = 'Explorer',
        executable = 'MAP EXPLORER',
        namespace="/",
        parameters = [config]

    )

    ld.add_action(node)
    return ld