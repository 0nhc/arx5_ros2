cmake_minimum_required(VERSION 3.8)
project(arx5_controller)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)

include_directories(
  thirdparty/arx5_sdk/include
)
set(arx5_sdk_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/arx5_sdk/lib")
set(arx5_sdk_LIB ${arx5_sdk_LIB_DIR}/libarx5_sdk.a)
link_directories(${arx5_sdk_LIB_DIR})

add_executable(controller src/arx5_controller.cpp)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(controller orocos-kdl Eigen3::Eigen ${arx5_sdk_LIB})
ament_target_dependencies(controller rclcpp sensor_msgs kdl_parser)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include src launch
  DESTINATION install/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ${arx5_sdk_LIB}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND OFF)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()