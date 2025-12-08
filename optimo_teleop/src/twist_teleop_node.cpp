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

#include "optimo_teleop/twist_teleop_node.hpp"

#include <chrono>
#include <functional>
#include <mutex>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "optimo_msgs/msg/pose_elbow.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

OptimoTeleop::OptimoTeleop(const rclcpp::NodeOptions & options) : Node("optimo_teleop", options)
{
  // Subscribe to current end-effector pose
  current_pose_subscriber_ = this->create_subscription<optimo_msgs::msg::PoseElbow>(
    "/optimo/ee_pose_current", 10,
    std::bind(&OptimoTeleop::current_pose_callback, this, std::placeholders::_1));

  // Initialize TF2 Buffer and Listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribe to twist-based input for controlling the end-effector
  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/optimo/servo/twist_cmd", 10,
    std::bind(&OptimoTeleop::twist_callback, this, std::placeholders::_1));

  // Publisher to send adjusted PoseElbow messages
  pose_pub_ =
    this->create_publisher<optimo_msgs::msg::PoseElbow>("/optimo/servo/ee_pose_desired", 10);

  // Initialize parameters
  position_scale_ = this->declare_parameter<double>("position_scale", 0.01);
  orientation_scale_ = this->declare_parameter<double>("orientation_scale", 0.01);

  RCLCPP_INFO(this->get_logger(), "OptimoTeleop node initialized.");
}

OptimoTeleop::~OptimoTeleop() {}

void OptimoTeleop::current_pose_callback(const optimo_msgs::msg::PoseElbow::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_pose_ = *msg;

  if (!initialized_) {
    desired_pose_ = current_pose_;
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Desired pose initialized from current pose.");
  }
}

void OptimoTeleop::twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);

  if (!initialized_) {
    RCLCPP_WARN(this->get_logger(), "Desired pose not initialized yet.");
    return;
  }

  // Get transformation from end-effector frame to world frame
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform(
      "world",                                       // Target frame (world frame)
      "end_effector",                                // Source frame (EE local frame)
      tf2::TimePointZero,                            // Get the latest transform
      tf2::Duration(std::chrono::milliseconds(100))  // Timeout
    );
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform from EE to world: %s", ex.what());
    return;
  }

  // Extract the rotation part of the transform (EE orientation in world frame)
  tf2::Quaternion q_ee_to_world;
  tf2::fromMsg(transformStamped.transform.rotation, q_ee_to_world);

  // The twist command is in world frame, but we need to apply it in EE frame
  // So we need to transform the twist from world to EE frame

  // 1. Transform linear velocity from world to EE frame
  tf2::Vector3 world_linear_vel(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

  // Rotate the linear velocity vector from world to EE frame
  // We need the inverse rotation (world to EE) which is the conjugate of the quaternion
  tf2::Vector3 ee_linear_vel = tf2::quatRotate(q_ee_to_world.inverse(), world_linear_vel);

  // 2. Transform angular velocity from world to EE frame
  tf2::Vector3 world_angular_vel(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
  tf2::Vector3 ee_angular_vel = tf2::quatRotate(q_ee_to_world.inverse(), world_angular_vel);

  // Apply transformed linear velocity to update position in world frame
  desired_pose_.pose.position.x += position_scale_ * world_linear_vel.x();
  desired_pose_.pose.position.y += position_scale_ * world_linear_vel.y();
  desired_pose_.pose.position.z += position_scale_ * world_linear_vel.z();

  // Create an incremental rotation quaternion for the world-frame rotation
  tf2::Quaternion dq;
  double delta_roll = orientation_scale_ * world_angular_vel.x();
  double delta_pitch = orientation_scale_ * world_angular_vel.y();
  double delta_yaw = orientation_scale_ * world_angular_vel.z();
  dq.setRPY(delta_roll, delta_pitch, delta_yaw);

  // Convert current orientation to tf2 quaternion
  tf2::Quaternion q_current;
  tf2::fromMsg(desired_pose_.pose.orientation, q_current);

  // Apply the rotation in world frame
  q_current = dq * q_current;
  q_current.normalize();  // Keep quaternion normalized

  // Convert back to geometry_msgs quaternion
  desired_pose_.pose.orientation = tf2::toMsg(q_current);

  // Preserve the elbow angle from the current pose
  desired_pose_.elbow_angle = current_pose_.elbow_angle;

  // Publish the updated desired PoseElbow
  pose_pub_->publish(desired_pose_);

  // Logging for debugging
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Published adjusted PoseElbow in world frame: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    desired_pose_.pose.position.x, desired_pose_.pose.position.y, desired_pose_.pose.position.z,
    desired_pose_.pose.orientation.x, desired_pose_.pose.orientation.y,
    desired_pose_.pose.orientation.z, desired_pose_.pose.orientation.w);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OptimoTeleop>();

  // Process subscriber callbacks
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}