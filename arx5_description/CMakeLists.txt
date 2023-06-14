cmake_minimum_required(VERSION 3.5)
project(arx5_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(arx5_display src/arx5_display.cpp)
target_include_directories(arx5_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arx5_display rclcpp sensor_msgs)

install(TARGETS arx5_display
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf meshes config launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
