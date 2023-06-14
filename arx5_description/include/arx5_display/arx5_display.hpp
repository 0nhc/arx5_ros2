#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ARX5Display : public rclcpp::Node
{
public:
  ARX5Display() : Node("arx5_display"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    subscription1_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_command", 10, std::bind(&ARX5Display::topic1_callback, this, _1));
    timer1_ = this->create_wall_timer(1ms, std::bind(&ARX5Display::timer1_callback, this));
  };
  
  std::vector<double> joint_states = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

private:
  void timer1_callback()
  {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "base_link";
    message.position = joint_states;
    message.name = {"base_link_to_link1", "link1_to_link2", "link2_to_link3", "link3_to_link4", "link4_to_link5", "link5_to_gripper_link1"};
    publisher_->publish(message);
  };
  void topic1_callback(const sensor_msgs::msg::JointState& msg)
  {
    joint_states = msg.position;
  };
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription1_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  size_t count_;
};