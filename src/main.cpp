#include "node.hpp"



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Sayer>(options));
  rclcpp::shutdown();
  return 0;
}
