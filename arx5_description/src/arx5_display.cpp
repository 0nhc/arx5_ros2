#include "arx5_display/arx5_display.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ARX5Display>());
  rclcpp::shutdown();
  return 0;
}