from geometry_msgs.msg import Pose2D
from gokart_msgs.msg import GokartState, SteerRef
from gokart_core_msgs.msg import Path
from rclpy.node import Node
from typing import Sequence, Optional

from .pure_pursuit import PurePursuit
from .speed_controller import SpeedController
from visualization_msgs.msg import Marker, MarkerArray
from gokart_cmd_msgs.msg import MotorsCmd
import numpy as np


class Metrics(Node):
    def __init__(self, node_name: str):
        """
        initialise controller
        :param node_name: str
        :param pure_pursuit: PurePursuit
        :param speed_controller: SpeedController
        """
        super(Metrics, self).__init__(node_name)
        self.gk_state = Pose2D()
        self.near_point = Pose2D()
        self.metric_dist_array = []
        self.path = []

        # subscribers
        self.create_subscription(Marker, '/kitt/near_point', self.update_near_point, 1)
        self.create_subscription(Marker, '/kitt/gk_pose', self.update_state, 1)
        self.create_subscription(Path, '/kitt/path/path_array', self.update_path, 1)
        self.create_subscription(GokartState, '/kitt/se2_localization/gokart_state', self.update_location, 1)

        #publishers
        self.center_path_array = self.create_publisher(MarkerArray, '/kitt/centre_path_array', 10)
        self.gk_pose_pub = self.create_publisher(Marker, '/kitt/gk_pose1', 10)


    def update_path(self, path_array: Path):
        self.path = path_array.path
        marker_array = MarkerArray()
        marker_array.markers = []
        for i in range(0, len(self.path)):
            marker_ = Marker()
            marker_.header.frame_id = "/map"
            marker_.type = marker_.SPHERE
            marker_.action = marker_.ADD
            marker_.ns = "est_pose_" + str(i)
            marker_.id = i
            marker_.pose.position.x = self.path[i].x
            marker_.pose.position.y = self.path[i].y
            marker_.scale.x = 0.2
            marker_.scale.y = 0.2
            marker_.scale.z = 0.2
            marker_.color.a = 0.5
            marker_.color.r = 0.0
            marker_.color.g = 1.0
            marker_.color.b = 1.0
            marker_array.markers.append(marker_)
        self.center_path_array.publish(marker_array)

    def update_location(self, msg: GokartState):
        print('t')
        self.gk_state = msg.pose2d
        gk_pose_marker_msg = self.gk_pose_marker()
        self.gk_pose_pub.publish(gk_pose_marker_msg)
        # self.find_near_point()

    def update_state(self, msg: Marker):
        """
        update gokart state
        :param msg: GokartState
        :return:
        """
        self.gk_state.x = msg.pose.position.x
        self.gk_state.y = msg.pose.position.y

    def find_near_point(self):
        near_point_dist = float('inf')
        for i in range(0, len(self.path)):
            dist = np.sqrt((self.gk_state.x - self.path[i].x) ** 2 + (self.gk_state.y - self.path[i].y) ** 2)
            if dist < near_point_dist:
                near_point_dist = dist
        print(near_point_dist)
        self.metric_dist_array.append(near_point_dist)
        avg = np.average(self.metric_dist_array)
        std = np.std(self.metric_dist_array)
        print(len(self.metric_dist_array))
        print('AVG = ', avg, 'STD = ', std)

    def update_near_point(self, msg: Marker):
        """
        update gokart path
        :param msg: Path
        :return:
        """
        self.near_point.x = msg.pose.position.x
        self.near_point.y = msg.pose.position.y
        if self.gk_state is not None:
            self.calc_metric()

    def calc_metric(self):
        """
        control loop and publishers
        :return:
        """
        dist = np.sqrt((self.gk_state.x - self.near_point.x) ** 2 + (self.gk_state.y - self.near_point.y) ** 2)
        print(dist)
        self.metric_dist_array.append(dist)
        avg = np.average(self.metric_dist_array)
        std = np.std(self.metric_dist_array)
        print(len(self.metric_dist_array))
        print('AVG = ', avg, 'STD = ', std)

    def gk_pose_marker(self):
        marker_ = Marker()
        marker_.header.frame_id = "/map"
        marker_.type = marker_.CUBE
        marker_.action = marker_.ADD
        marker_.pose.position.x = self.gk_state.x
        marker_.pose.position.y = self.gk_state.y
        marker_.scale.x = 1.0
        marker_.scale.y = 1.0
        marker_.scale.z = 1.0
        marker_.color.a = 0.5
        marker_.color.r = 0.0
        marker_.color.g = 1.0
        marker_.color.b = 0.0
        return marker_
