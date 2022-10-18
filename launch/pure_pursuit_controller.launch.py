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
    steer_controller_param = os.path.join(
        get_package_share_directory('pure_pursuit_controller'),
        'param',
        'steering_controller_params.yaml'
    )

    pure_pursuit_controller = Node(
        package="pure_pursuit_controller",
        name="pure_pursuit_node",
        output="screen",
        executable="pure_pursuit_node",
        parameters=[pure_pursuit_controller_param],
        remappings=[
            ('~/gokart_state', 'se2_localization/gokart_state'),
            ('~/steer_ref', 'steering_controller/steer_ref'),
            ('~/motors_cmd', 'autonomous_control/motors_cmd'),
            ('~/start_controller', 'start_controller'),
            ('~/path_array', 'path/path_array'),
            ('~/gk_pose', 'gk_pose'),
            ('~/near_point', 'near_point'),
            ('~/goal_point', 'goal_point'),
        ],
        namespace=robot_namespace
    )

    steering_controller = Node(
        package="steering_controller",
        name="steering_controller_node",
        output="screen",
        executable="steering_controller_node",
        parameters=[steer_controller_param],
        remappings=[
            ('~/steer_ref', 'steering_controller/steer_ref'),
            ('~/steer_obs', 'observations/steer_obs'),
            ('~/steer_cmd', 'autonomous_control/steer_cmd'),
        ],
        namespace=robot_namespace
    )

    ld.add_action(steering_controller)
    ld.add_action(pure_pursuit_controller)
    return ld
