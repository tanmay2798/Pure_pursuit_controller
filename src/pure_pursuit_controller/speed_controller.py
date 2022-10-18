from abc import ABC, abstractmethod
from typing import Optional
from .parameters import Parameters


class SpeedControllerAbc(ABC):

    @abstractmethod
    def controller(self, v_current: float, v_desired: float, time: float) -> float:
        ...


class SpeedController(SpeedControllerAbc):

    def __init__(self):
        self.parameters = Parameters()

        # for speed controller
        self.dt: Optional[float] = None
        self.elapsed_time: float = 0
        self.error_sum: float = 0
        self.v_error: float = 0

        # parameters from param file
        self.kp: float = self.parameters.params['vel_pid']['kP']
        self.ki: float = self.parameters.params['vel_pid']['kI']

    def controller(self, v_current: float, v_desired: float, time: float) -> float:
        """
        speed controller
        :param time:
        :param v_current:float
        :param v_desired:float
        :return motor_output:MotorsCmd
        """
        self.v_error = v_desired - v_current
        if self.dt is None:
            self.dt = 0.1
        else:
            self.dt = (time - self.elapsed_time)
        self.elapsed_time = time
        self.error_sum = self.error_sum + self.v_error * self.dt
        steering_correction = (self.kp * self.v_error + self.ki * self.error_sum)
        print('speed', steering_correction)
        return steering_correction
