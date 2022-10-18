import rclpy
from gokart_core_msgs.msg import Path
from geometry_msgs.msg import Pose2D

from .structures import Track
from .tracks_loader import tracks

from .parameters import Parameters

curve = tracks["winti_006"]
track = Track("winti_006", curve.spline, None, 1)
centreline, tangents = track.get_n_points(1000)
places_array = []

for i in range(0, len(centreline)):
    print('ppp', i)
    path_point = Pose2D()
    path_point.x = centreline[i][0]
    path_point.y = centreline[i][1]
    path_point.theta = tangents[i]
    places_array.append(path_point)

path_array = Path()
rclpy.init(args=None)
node = rclpy.create_node('data')
parameters = Parameters()

if parameters.params['mode'] == 'reverse':
    places_array.reverse()

path_array.path = places_array
state_publisher = node.create_publisher(Path, '~/path_array', 10)
while True:
    state_publisher.publish(path_array)
    
rclpy.spin(node)
state_publisher.destroy_node()
rclpy.shutdown()
