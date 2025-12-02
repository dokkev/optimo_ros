// Copyright 2024 Roboligent, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPTIMO_TELEOP__JOINT_TELEOP_NODE_HPP_
#define OPTIMO_TELEOP__JOINT_TELEOP_NODE_HPP_

#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class OptimoJointTeleop : public rclcpp::Node
{
public:
  explicit OptimoJointTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~OptimoJointTeleop();

private:
  void current_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void joint_velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_state_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_subscriber_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // State variables
  sensor_msgs::msg::JointState current_joint_state_;
  sensor_msgs::msg::JointState desired_joint_state_;
  std::mutex joint_mutex_;
  bool initialized_ = false;

  // Parameters
  double velocity_scale_;  // Scale factor for joint velocity commands
  int num_joints_;  // Number of joints (default 7)
};

#endif  // OPTIMO_TELEOP__JOINT_TELEOP_NODE_HPP_
