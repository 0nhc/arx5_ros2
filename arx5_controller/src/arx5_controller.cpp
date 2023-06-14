#include "arx5_controller/arx5_controller.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ARX5Controller>());
  rclcpp::shutdown();
  return 0;
}