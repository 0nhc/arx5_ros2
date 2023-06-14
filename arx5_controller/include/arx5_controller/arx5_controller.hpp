#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <arx5_base/hardware_interface.h>
#include <arx5_base/joint_trajectories.h>
#include <arx5_base/kinematics_dynamics.h>
#include <arx5_base/planner.h>
#include <arx5_base/rate.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ARX5Controller : public rclcpp::Node
{
public:
  ARX5Controller() : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);
    subscription1_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&ARX5Controller::topic1_callback, this, _1));
    subscription2_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "target_pose", 10, std::bind(&ARX5Controller::topic2_callback, this, _1));
    timer1_ = this->create_wall_timer(1ms, std::bind(&ARX5Controller::timer1_callback, this));
    timer2_ = this->create_wall_timer(1000ms, std::bind(&ARX5Controller::timer2_callback, this));

    joint_trajectories.update(planner.getTraj(target_pose));
  };

  JointTrajectories joint_trajectories;
  KinematicsDynamics kinematics_dynamics;
  Planner planner;

  std::vector<double> target_pose = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  std::vector<double> joint_states = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  std::vector<double> joint_command = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  std::vector<double> pose_states = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  bool received_joint_states = false;

private:
  void timer1_callback()
  {
    joint_command = joint_trajectories.front();
    joint_trajectories.pop();
    joint_command = kinematics_dynamics.solveIK(joint_command);
    for (int i = 0; i < 6; i++)
    {
      joint_command[i] = pi2pi(joint_command[i]);
    }

    auto message = sensor_msgs::msg::JointState();
    message.name.push_back("base_link_to_link1");
    message.name.push_back("link1_to_link2");
    message.name.push_back("link2_to_link3");
    message.name.push_back("link3_to_link4");
    message.name.push_back("link4_to_link5");
    message.name.push_back("link5_to_gripper_link1");
    message.position = joint_command;
    publisher_->publish(message);

    std::cout << "sim: " << received_joint_states << ", joint_command: " << joint_command[0] << ", " << joint_command[1]
              << ", " << joint_command[2] << ", " << joint_command[3] << ", " << joint_command[4] << ", "
              << joint_command[5] << std::endl;

    if (received_joint_states == false)
    {
      joint_states = joint_command;
    }
    kinematics_dynamics.updateJointStates(joint_states);
    pose_states = kinematics_dynamics.solveFK(joint_states);
    planner.updatePoseStates(pose_states);
  };
  void timer2_callback()
  {
    joint_trajectories.update(planner.getTraj(target_pose));
  };
  void topic1_callback(const sensor_msgs::msg::JointState& msg)
  {
    received_joint_states = true;
    for (int i = 0; i < 6; i++)
    {
      joint_states[i] = pi2pi(msg.position[i]);
    }
    joint_states[4] = -joint_states[4];
  };
  void topic2_callback(const std_msgs::msg::Float64MultiArray& msg)
  {
    if (msg.data.size() == 6)
    {
      target_pose = msg.data;
    }
  };
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription1_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription2_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  size_t count_;

  double pi2pi(double angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }
    return angle;
  };
};