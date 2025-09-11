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

#ifndef OPTIMO_TELEOP__TELEOP_NODE_HPP_
#define OPTIMO_TELEOP__TELEOP_NODE_HPP_

#include <mutex>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "optimo_msgs/msg/pose_elbow.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class OptimoTeleop : public rclcpp::Node
{
public:
  explicit OptimoTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~OptimoTeleop();

private:
  void current_pose_callback(const optimo_msgs::msg::PoseElbow::SharedPtr msg);
  void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // Subscribers
  rclcpp::Subscription<optimo_msgs::msg::PoseElbow>::SharedPtr current_pose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;

  // Publisher
  rclcpp::Publisher<optimo_msgs::msg::PoseElbow>::SharedPtr pose_pub_;

  // TF2 objects
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State variables
  optimo_msgs::msg::PoseElbow current_pose_;
  optimo_msgs::msg::PoseElbow desired_pose_;
  std::mutex pose_mutex_;
  bool initialized_ = false;

  // Parameters
  double position_scale_;  // Scale factor for position control
  double orientation_scale_;  // Scale factor for orientation control
};

#endif  // OPTIMO_TELEOP__TELEOP_NODE_HPP_