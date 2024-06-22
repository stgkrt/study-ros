import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("hello")
    node.get_logger().info("Hello, ROS2 world!")

    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()