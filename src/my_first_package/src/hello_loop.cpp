# include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hello_loop");
    RCLCPP_INFO(node->get_logger(), "Hello, Before loop ROS2 world!");

    rclcpp::WallRate loop(1);
    while (rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "Hello, ROS2 world!");
        loop.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}