from rclpy.node import Node


class Parameters(Node):

    def __init__(self):
        super().__init__('parameters')

        # Declare parameters
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('vel_pid.kP', 0),
                                    ('vel_pid.kI', 0),
                                    ('vel_pid.kD', 0),
                                    ('look_ahead_distance', 0),
                                    ('steering_factor', 0),
                                    ('min_distance', 0),
                                    ('v_des', 0),
                                    ('look_ahead_distance_reverse', 0),
                                    ('min_distance_reverse', 0),
                                    ('v_des_reverse', 0),
                                    ('mode', 'deadlock')
                                ]
                                )

        # Set parameters
        self.params = {
            'vel_pid': {
                'kP': self.get_parameter('vel_pid.kP').get_parameter_value().double_value,
                'kI': self.get_parameter('vel_pid.kI').get_parameter_value().double_value,
                'kD': self.get_parameter('vel_pid.kD').get_parameter_value().double_value,
            },
            'look_ahead_distance': self.get_parameter('look_ahead_distance').get_parameter_value().double_value,
            'min_distance': self.get_parameter('min_distance').get_parameter_value().double_value,
            'steering_factor': self.get_parameter('steering_factor').get_parameter_value().double_value,
            'v_des': self.get_parameter('v_des').get_parameter_value().double_value,
            'look_ahead_distance_reverse': self.get_parameter('look_ahead_distance_reverse').get_parameter_value().double_value,
            'min_distance_reverse': self.get_parameter('min_distance_reverse').get_parameter_value().double_value,
            'v_des_reverse': self.get_parameter('v_des_reverse').get_parameter_value().double_value,
            'mode': self.get_parameter('mode').get_parameter_value().string_value
        }

        self.get_logger().info(str(self.params))

        # TODO: implement dynamic updates
