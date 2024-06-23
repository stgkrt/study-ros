import sys

import rclpy
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("my_subscriber")
    subscription = node.create_subscription(
        String,
        "greeting",
        lambda msg: node.get_logger().info('I heard: "%s"' % msg.data),
        10
    )
    subscription
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()