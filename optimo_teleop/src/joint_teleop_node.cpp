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

#include "optimo_teleop/joint_teleop_node.hpp"

#include <chrono>
#include <functional>
#include <mutex>

using namespace std::chrono_literals;

OptimoJointTeleop::OptimoJointTeleop(const rclcpp::NodeOptions & options)
: Node("optimo_joint_teleop", options)
{
  // Initialize parameters
  velocity_scale_ = this->declare_parameter<double>("velocity_scale", 0.01);
  num_joints_ = this->declare_parameter<int>("num_joints", 7);

  // Subscribe to current joint state
  current_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/optimo/joint_states", 10,
    std::bind(&OptimoJointTeleop::current_joint_state_callback, this, std::placeholders::_1));

  // Subscribe to joint velocity commands (7 values for 7 joints)
  joint_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/optimo/joint_teleop/velocity_cmd", 10,
    std::bind(&OptimoJointTeleop::joint_velocity_callback, this, std::placeholders::_1));

  // Publisher to send desired joint states
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("/optimo/joint_teleop/desired_joint_state", 10);

  RCLCPP_INFO(this->get_logger(), "OptimoJointTeleop node initialized.");
  RCLCPP_INFO(this->get_logger(), "  velocity_scale: %.4f", velocity_scale_);
  RCLCPP_INFO(this->get_logger(), "  num_joints: %d", num_joints_);
}

OptimoJointTeleop::~OptimoJointTeleop() {}

void OptimoJointTeleop::current_joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_mutex_);
  current_joint_state_ = *msg;

  if (!initialized_ && msg->position.size() >= static_cast<size_t>(num_joints_)) {
    // Initialize desired_joint_state from current state
    desired_joint_state_ = current_joint_state_;
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Desired joint state initialized from current state.");
    
    // Log initial positions with joint names
    std::string pos_str;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      pos_str += msg->name[i] + ": " + std::to_string(msg->position[i]);
      if (i < msg->name.size() - 1) pos_str += ", ";
    }
    RCLCPP_INFO(this->get_logger(), "Initial joint state: %s", pos_str.c_str());
  }
}

void OptimoJointTeleop::joint_velocity_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_mutex_);

  if (!initialized_) {
    RCLCPP_WARN(this->get_logger(), "Desired joint state not initialized yet.");
    return;
  }

  if (msg->data.size() != static_cast<size_t>(num_joints_)) {
    RCLCPP_WARN(
      this->get_logger(), 
      "Expected %d joint velocities, got %zu. Ignoring command.", 
      num_joints_, msg->data.size());
    return;
  }

  // Apply incremental changes to desired joint positions
  // The velocity commands are in the order [joint1, joint2, ..., joint7]
  // But the actual joint_state has scrambled names, so we need to map correctly
  
  // Create a map from joint name to velocity command index
  for (size_t i = 0; i < desired_joint_state_.name.size(); ++i) {
    // Extract joint number from name (e.g., "joint2" -> 2)
    std::string joint_name = desired_joint_state_.name[i];
    if (joint_name.size() >= 6 && joint_name.substr(0, 5) == "joint") {
      int joint_num = std::stoi(joint_name.substr(5)) - 1;  // Convert to 0-based index
      
      if (joint_num >= 0 && joint_num < num_joints_) {
        desired_joint_state_.position[i] += velocity_scale_ * msg->data[joint_num];
      }
    }
  }

  // Update timestamp
  desired_joint_state_.header.stamp = this->now();

  // Publish the updated desired joint state
  joint_state_pub_->publish(desired_joint_state_);

  // Logging for debugging (throttled to 1 Hz)
  // Build a string showing joint name and position for clearer output
  static int log_count = 0;
  if (++log_count % 50 == 0) {  // Log every 50 messages (~1 Hz at 50 Hz)
    std::string state_str;
    for (size_t i = 0; i < desired_joint_state_.name.size(); ++i) {
      state_str += desired_joint_state_.name[i] + ":" + 
                   std::to_string(desired_joint_state_.position[i]).substr(0, 6);
      if (i < desired_joint_state_.name.size() - 1) state_str += " ";
    }
    RCLCPP_INFO(this->get_logger(), "Desired: %s", state_str.c_str());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OptimoJointTeleop>();

  // Process subscriber callbacks
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
