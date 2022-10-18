from abc import ABC, abstractmethod
from typing import Sequence, Optional

from geometry_msgs.msg import Pose2D
from visualization_msgs.msg import Marker
from .speed_controller import SpeedController
from gokart_core_msgs.msg import Path
from .parameters import Parameters
import numpy as np


class PurePursuitAbc(ABC):

    @abstractmethod
    def update_path(self, path_array: Path):
        ...

    @abstractmethod
    def update_location(self, location: Pose2D):
        ...

    @abstractmethod
    def update_velocity(self, velocity: Pose2D):
        ...

    @abstractmethod
    def find_nearest_point_index_on_path(self, path: Sequence[Pose2D]) -> int:
        ...

    @abstractmethod
    def find_goal_point(self, path: Sequence[Pose2D]) -> Pose2D:
        ...

    @abstractmethod
    def is_aligned(self) -> bool:
        ...

    @abstractmethod
    def get_curvature(self) -> float:
        ...

    @abstractmethod
    def compute_steering_angle(self) -> float:
        ...

    @abstractmethod
    def compute_motors_cmd(self, speed_controller: SpeedController, run_time: float) -> float:
        ...

    @abstractmethod
    def update_parameters(self):
        ...


class PurePursuit(PurePursuitAbc):

    def __init__(self):
        """
        initialise pure_pursuit control loop
        :param
        """
        self.parameters = Parameters()                          # initialise parameters

        # for pure pursuit controller
        self.path: Sequence[Pose2D] = []                        # array of the path to be followed
        self.gk_state: Optional[Pose2D] = None  # gokart state
        self.start_index: int = 0  # index of path array gokart starts from
        self.gk_velocity: float = 0  # gokart velocity
        self.look_ahead_point: Optional[Pose2D] = None  # look ahead point on the path
        self.near_point_index_on_path: int = 0  # nearest point on path's index
        self.goal_point_in_vehicle_frame: Pose2D = Pose2D()  # target point in vehicle frame
        self.curvature: float = 0  # curvature of the point from current position to be reached
        self.gk_length: float = 1.19  # gokart length
        self.steering_angle: float = 0  # steering angle
        self.is_start_controller: bool = False  # checks if controller is started or not

        # from take home functionality
        self.gk_state_path_history: Sequence[
            Pose2D] = []  # array of gk state for retracing the path to reach centre of track in take home functionality
        self.near_point_on_path_dist: float = float("inf")  # nearest point on main path's index
        self.gk_moving_forward = None  # checks if gokart is moving forward
        self.initial_direction = 0  # checks gokart initial movement

        # parameters from param file
        self.mode: str = self.parameters.params['mode']
        self.v_desired = 0
        self.look_ahead_distance: float = 0
        self.min_distance: float = 0
        self.update_parameters()

        # metrics
        self.metric_dist_array = []

        # todo convert gokart poses to numpy arrays

    def update_path(self, path_array: Path):
        """
        store points in array
        :param path_array: Path
        :return:
        """
        self.path = path_array

    def update_location(self, location: Pose2D):
        """
        update current location
        :param location: Pose2D
        :return:
        """
        self.gk_state = location
        if self.is_start_controller is False:               # to store gk states for revering in take home functionality
            self.gk_state_path_history.append(self.gk_state)

        if self.look_ahead_point is None and self.is_start_controller is True:       # provide initial look ahead point in start direction
            self.look_ahead_point = Pose2D()
            angle = self.gk_state.theta
            rotation = np.matmul(
                np.array([[np.round(np.cos(angle), 1), -np.round(np.sin(angle), 1)],
                          [np.round(np.sin(angle), 1), np.round(np.cos(angle), 1)]]),
                np.array([[self.initial_direction*self.look_ahead_distance], [0.0]]))

            self.look_ahead_point.x = float(rotation[0]) + self.gk_state.x
            self.look_ahead_point.y = float(rotation[1]) + self.gk_state.y

    def update_velocity(self, velocity: Pose2D):
        """

        :param velocity: Pose2D
        :return:
        """
        self.gk_velocity = velocity.x

    def find_nearest_point_index_on_path(self, path: Sequence[Pose2D]) -> int:
        """
        find nearest point on path to gokart
        :return:
        """
        near_point_dist = float('inf')
        self.near_point_index_on_path = 0
        for i in range(0, len(path)):
            dist = np.sqrt((self.gk_state.x - path[i].x) ** 2 + (self.gk_state.y - path[i].y) ** 2)
            if dist < near_point_dist:
                near_point_dist = dist
                self.near_point_index_on_path = i
        self.metric_dist_array.append(near_point_dist)
        return self.near_point_index_on_path

    def find_goal_point(self, path: Sequence[Pose2D]) -> Pose2D:
        """
        find goal point on path
        :return: Pose2D
        """
        dist_array = []
        pose = []

        dist = np.sqrt(
            (self.look_ahead_point.x - self.gk_state.x) ** 2 + (self.look_ahead_point.y - self.gk_state.y) ** 2)

        if dist < self.min_distance:
            for i in range(self.near_point_index_on_path, self.near_point_index_on_path + 20):
                i = i % len(path)
                dist = np.sqrt((self.gk_state.x - path[i].x) ** 2 + (self.gk_state.y - path[i].y) ** 2)
                dist_array.append(abs(dist - self.look_ahead_distance))
                pose.append(path[i])

            print('dist', min(dist_array))
            look_ahead_point_index = dist_array.index(min(dist_array))
            self.look_ahead_point.x = pose[look_ahead_point_index].x
            self.look_ahead_point.y = pose[look_ahead_point_index].y

        return self.look_ahead_point

    def is_aligned(self) -> bool:
        """
        check if gokart is in centre and aligned and can be changed from reverse mode to forward mode
        :return: bool
        """
        self.near_point_on_path_dist = float('inf')
        min_index = 0
        theta = 0
        for i in range(0, len(self.path)):
            theta = self.gk_state.theta
            if self.gk_state.theta < 0:      # as gk state gives angle from [-pi, pi]
                theta = theta + np.pi
            dist = np.sqrt(
                (self.gk_state.x - self.path[i].x) ** 2 + (self.gk_state.y - self.path[i].y) ** 2 + (theta - self.path[i].theta) ** 2)
            if dist < self.near_point_on_path_dist:
                self.near_point_on_path_dist = dist
                min_index = i

        print(round((self.gk_state.x - self.path[min_index].x), 2), '         ',
              round((self.gk_state.y - self.path[min_index].y), 2),
              '          ', round(self.gk_state.theta, 2), round(theta, 2), round(self.path[min_index].theta, 2))

        if self.near_point_on_path_dist < 0.8:   # threshold distance
            self.mode = 'forward'
            self.update_parameters()
            return True
        else:
            return False

    def get_curvature(self) -> float:
        """
        find curvature
        :return: float
        """
        self.goal_point_in_vehicle_frame.x = self.look_ahead_point.x - self.gk_state.x
        self.goal_point_in_vehicle_frame.y = self.look_ahead_point.y - self.gk_state.y
        self.goal_point_in_vehicle_frame.theta = self.gk_state.theta

        angle = -self.gk_state.theta
        rotation = np.matmul(
            np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]),
            np.array([[self.goal_point_in_vehicle_frame.x], [self.goal_point_in_vehicle_frame.y]]))
        self.goal_point_in_vehicle_frame.y = float(rotation[1])
        perpendicular_dist = self.goal_point_in_vehicle_frame.y
        self.curvature = (2 * perpendicular_dist / self.look_ahead_distance ** 2)
        return self.curvature

    def compute_steering_angle(self) -> float:
        """
        gives steering angle
        :return: float
        """
        # put here the logic
        self.steering_angle = np.arctan(self.gk_length * self.curvature)
        return self.parameters.params['steering_factor'] * self.steering_angle

    def compute_motors_cmd(self, speed_controller: SpeedController, run_time: float) -> float:
        """
        gives steering angle
        :param run_time: float
        :param speed_controller: SpeedController
        :return: float
        """
        # put here the logic
        pid_values = speed_controller.controller(self.gk_velocity, self.v_desired, run_time)
        if self.near_point_index_on_path > len(self.path) - 5:
            return 0.0
        else:
            return pid_values

    def calculate_metric(self):
        print(self.metric_dist_array[-1])
        avg = np.average(self.metric_dist_array)
        std = np.std(self.metric_dist_array)
        print('AVG = ', avg, 'STD = ', std)

    def update_parameters(self):
        """

        :return:
        """
        if self.mode == 'reverse' or self.mode == 'deadlock':
            self.min_distance = self.parameters.params['min_distance_reverse']
            self.v_desired = self.parameters.params['v_des_reverse']
            self.look_ahead_distance = self.parameters.params['look_ahead_distance_reverse']
            self.initial_direction = -1
            self.gk_moving_forward = False
        else:
            self.v_desired = self.parameters.params['v_des']
            self.look_ahead_distance = self.v_desired + 1
            self.min_distance = self.look_ahead_distance*0.55 + 0.76
            self.initial_direction = 1
            self.gk_moving_forward = True

    def near_marker(self):
        marker_ = Marker()
        marker_.header.frame_id = "/map"
        marker_.type = marker_.CUBE
        marker_.action = marker_.ADD
        marker_.pose.position.x = self.path[self.near_point_index_on_path].x
        marker_.pose.position.y = self.path[self.near_point_index_on_path].y
        marker_.scale.x = 1.0
        marker_.scale.y = 1.0
        marker_.scale.z = 1.0
        marker_.color.a = 0.5
        marker_.color.r = 1.0
        marker_.color.g = 0.0
        marker_.color.b = 0.0
        return marker_

    def goal_marker(self):
        marker_ = Marker()
        marker_.header.frame_id = "/map"
        marker_.type = marker_.CUBE
        marker_.action = marker_.ADD
        marker_.pose.position.x = self.look_ahead_point.x
        marker_.pose.position.y = self.look_ahead_point.y
        marker_.scale.x = 1.0
        marker_.scale.y = 1.0
        marker_.scale.z = 1.0
        marker_.color.a = 0.5
        marker_.color.r = 0.0
        marker_.color.g = 0.0
        marker_.color.b = 1.0
        return marker_

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