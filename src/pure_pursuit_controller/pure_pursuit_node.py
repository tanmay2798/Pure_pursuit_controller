import rclpy
from .pure_pursuit_controller import PurePursuitController
from .pure_pursuit import PurePursuit
from .speed_controller import SpeedController
from .metrics import Metrics


def main():
    # todo start up node an make it spin
    rclpy.init(args=None)
    pure_pursuit = PurePursuit()
    speed_controller = SpeedController()
    # pure_pursuit_controller = PurePursuitController('pure_pursuit', pure_pursuit, speed_controller)
    metrics = Metrics('metrics')
    rclpy.spin(metrics)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
