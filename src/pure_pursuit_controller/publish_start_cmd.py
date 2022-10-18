import rclpy
from std_msgs.msg import Bool

msg = Bool()
msg.data = True
rclpy.init(args=None)
node = rclpy.create_node('data')
state_publisher = node.create_publisher(Bool, '~/start_controller', 10)
while True:
    state_publisher.publish(msg)
rclpy.spin(node)
state_publisher.destroy_node()
rclpy.shutdown()
