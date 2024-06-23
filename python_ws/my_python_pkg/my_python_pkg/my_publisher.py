import sys

import rclpy
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("my_publisher")
    publisher = node.create_publisher(String, "greeting", 10)

    msg = String()
    i = 0
    def timer_callback():
        nonlocal i
        msg.data = 'Hello World: %d' %i
        i += 1
        node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)
    
    while rclpy.ok():
        timer_period = 1.0  # seconds
        timer = node.create_timer(timer_period, timer_callback)
        timer  # Quiet flake8 warnings about unused variable
        rclpy.spin(node)
    
    rclpy.shutdown()

if __name__=="__main__":
    main()