import rclpy
from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("turtle_controller")
    publisher = node.create_publisher(Twist, "/turtle1/cmd_vel", 1)

    msg = Twist()
    timer_period = 1.0
    def timer_callback():
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        node.get_logger().info('Publishing linear x:"%s"' % msg.linear.x)
        node.get_logger().info('Publishing angular z:"%s"' % msg.angular.z)
        publisher.publish(msg)

    while rclpy.ok():
        timer = node.create_timer(timer_period, timer_callback)
        timer
        rclpy.spin(node)
    
    rclpy.shutdown()

if __name__=="__main__":
    main()

   