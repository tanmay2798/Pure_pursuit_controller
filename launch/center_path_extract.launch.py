import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from gokart_commons.utils import get_robot_namespace


def generate_launch_description():
    robot_namespace = '/kitt'
    # robot_namespace = get_robot_namespace()
    ld = LaunchDescription()

    pure_pursuit_controller_param = os.path.join(
        get_package_share_directory('pure_pursuit_controller'),
        'param',
        'pure_pursuit_params.yaml'
    )

    track_extractor = Node(
        package="pure_pursuit_controller",
        name="track_node",
        output="screen",
        executable="publish_path",
        parameters=[pure_pursuit_controller_param],
        remappings=[
            ('~/path_array', 'path/path_array'),
        ],
        namespace=robot_namespace
    )

    ld.add_action(track_extractor)
    return ld
