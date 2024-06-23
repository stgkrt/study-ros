import rclpy
from time import sleep

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("hello_loop")

    # keyboard 操作があるまでループ
    while rclpy.ok():
        node.get_logger().info("Hello, ROS2 world!")
        #rosのsleepがわからなかった。クラスならcallbackがあるっぽい
        sleep(1)

    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()