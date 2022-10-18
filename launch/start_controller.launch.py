from launch import LaunchDescription
from launch_ros.actions import Node
from gokart_commons.utils import get_robot_namespace


def generate_launch_description():
    # robot_namespace = '/kitt'
    robot_namespace = get_robot_namespace()
    ld = LaunchDescription()

    start_controller = Node(
        package="pure_pursuit_controller",
        name="start_controller_node",
        output="screen",
        executable="publish_start_cmd",
        remappings=[
            ('~/start_controller', 'start_controller'),
        ],
        namespace=robot_namespace
    )

    ld.add_action(start_controller)
    return ld
