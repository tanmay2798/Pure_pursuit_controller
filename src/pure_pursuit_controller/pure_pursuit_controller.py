from gokart_msgs.msg import GokartState, SteerRef
from gokart_core_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool

from .pure_pursuit import PurePursuit
from .speed_controller import SpeedController
from visualization_msgs.msg import Marker
from gokart_cmd_msgs.msg import MotorsCmd
import numpy as np


class PurePursuitController(Node):
    def __init__(self, node_name: str, pure_pursuit: PurePursuit, speed_controller: SpeedController):
        """
        initialise controller
        :param node_name: str
        :param pure_pursuit: PurePursuit
        :param speed_controller: SpeedController
        """
        super(PurePursuitController, self).__init__(node_name)
        self.angle_ref_old: float = 0

        # for take home functionality
        self.is_reversed: bool = False  # tells if gokart has reversed completely from deadlock
        self.is_started: bool = False   # tells if gokart is started by controller or not

        # publishers
        self.steer_pub = self.create_publisher(SteerRef, '~/steer_ref', 10)
        self.near_marker_pub = self.create_publisher(Marker, '~/near_point', 10)
        self.gk_pose_pub = self.create_publisher(Marker, '~/gk_pose', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '~/goal_point', 10)
        self.vel_pub = self.create_publisher(MotorsCmd, '~/motors_cmd', 10)

        # subscribers
        self.create_subscription(Path, '~/path_array', self.update_path, 10)
        self.create_subscription(GokartState, '~/gokart_state', self.update_state, 10)
        self.create_subscription(Bool, '~/start_controller', self.start_controller, 10)

        # Class objects
        self.pure_pursuit = pure_pursuit
        self.speed_controller = speed_controller

    def update_state(self, msg: GokartState):
        """
        update gokart state
        :param msg: GokartState
        :return:
        """
        self.pure_pursuit.update_location(msg.pose2d)
        self.pure_pursuit.update_velocity(msg.pose2d_dot)
        if not self.is_started and len(self.pure_pursuit.path) > 0:
            self.pure_pursuit.start_index = self.pure_pursuit.find_nearest_point_index_on_path(self.pure_pursuit.path)
            self.is_started = True

        if len(self.pure_pursuit.path) > 0 and self.pure_pursuit.is_start_controller is True:
            self.control()
        elif len(self.pure_pursuit.path) == 0:
            motor_msg = MotorsCmd()
            motor_msg.header.stamp = self.get_clock().now().to_msg()
            motor_msg.left = 0.0
            motor_msg.right = 0.0
            self.vel_pub.publish(motor_msg)

    def update_path(self, msg: Path):
        """
        update gokart path
        :param msg: Path
        :return:
        """
        if len(self.pure_pursuit.path) == 0:
            self.pure_pursuit.update_path(msg.path)

    def start_controller(self, msg: Bool):
        """
        start controller
        :param msg: Bool
        :return:
        """
        self.pure_pursuit.is_start_controller = msg.data
        if self.is_reversed is False:
            motor_msg = MotorsCmd()
            motor_msg.header.stamp = self.get_clock().now().to_msg()
            motor_msg.left = 0.0
            motor_msg.right = 0.0
            self.vel_pub.publish(motor_msg)
            steer_ref_msg = SteerRef()
            steer_ref_msg.header.stamp = self.get_clock().now().to_msg()
            steer_ref_msg.angle = 0.0
            self.steer_pub.publish(steer_ref_msg)
            self.pure_pursuit.gk_state_path_history.reverse()
            self.is_reversed = True

    def control(self):
        """
        control loop and publishers
        :return:
        """
        # todo put here code of main callback(this is the one you bind to a subscription)
        # todo here wrap/ unwrap ros stuff and use self.pure_pursuit
        print(self.pure_pursuit.gk_moving_forward, self.pure_pursuit.near_point_on_path_dist)
        if self.pure_pursuit.mode == 'deadlock':      # to check if gokart is aligned in take home functionality mode
            near_point_index = self.pure_pursuit.find_nearest_point_index_on_path(self.pure_pursuit.gk_state_path_history)
            self.pure_pursuit.find_goal_point(self.pure_pursuit.gk_state_path_history)
            self.pure_pursuit.is_aligned()
            if near_point_index > len(self.pure_pursuit.gk_state_path_history)-5:
                self.pure_pursuit.gk_moving_forward = True
                self.pure_pursuit.mode = 'forward'
                self.pure_pursuit.update_parameters()
        else:
            self.pure_pursuit.find_nearest_point_index_on_path(self.pure_pursuit.path)
            self.pure_pursuit.find_goal_point(self.pure_pursuit.path)
        self.pure_pursuit.calculate_metric()
        self.pure_pursuit.get_curvature()
        angle_ref = self.pure_pursuit.compute_steering_angle()
        pid_output = self.pure_pursuit.compute_motors_cmd(self.speed_controller, self.get_clock().now().nanoseconds)
        steer_ref_msg = SteerRef()
        angles = np.linspace(self.angle_ref_old, angle_ref, num=1)

        print(angles)
        print('mode', self.pure_pursuit.mode)

        for i in angles:
            steer_ref_msg.header.stamp = self.get_clock().now().to_msg()
            steer_ref_msg.angle = np.clip(i, -1.4, 1.4)
            self.steer_pub.publish(steer_ref_msg)

        self.angle_ref_old = angle_ref
        motor_msg = MotorsCmd()
        motor_msg.header.stamp = self.get_clock().now().to_msg()
        motor_msg.left = np.clip(pid_output, -0.5, 0.5)
        motor_msg.right = np.clip(pid_output, -0.5, 0.5)

        near_marker_msg = self.pure_pursuit.near_marker()
        goal_marker_msg = self.pure_pursuit.goal_marker()
        gk_pose_marker_msg = self.pure_pursuit.gk_pose_marker()

        near_marker_msg.header.stamp = self.get_clock().now().to_msg()
        goal_marker_msg.header.stamp = self.get_clock().now().to_msg()
        gk_pose_marker_msg.header.stamp = self.get_clock().now().to_msg()

        self.gk_pose_pub.publish(gk_pose_marker_msg)
        self.goal_marker_pub.publish(goal_marker_msg)
        self.near_marker_pub.publish(near_marker_msg)
        self.vel_pub.publish(motor_msg)

        print("")
