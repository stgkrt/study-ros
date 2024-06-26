#include "rclcpp/rclcpp.hpp"
#include "my_first_package/simple_subscriber.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}