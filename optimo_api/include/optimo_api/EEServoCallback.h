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

#ifndef OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_API_INCLUDE_OPTIMO_API_SERVOCALLBACK_H_
#define OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_API_INCLUDE_OPTIMO_API_SERVOCALLBACK_H_

#include <future>

#include <rl/controller/AbstractCallback.h>
#include <rl/model/ModelTypes.h>
#include <rl/model/ModelUtil.h>
#include <rl/trajectory/CartesianTrajectory.h>
#include <rl/trajectory/JointTrajectory.h>
#include <rl/util/DelayedLerp.h>
#include <rl/util/Logger.h>
#include <rl/util/Timer.h>
#include <rl/util/Vector.h>

#include <optimo_msgs/msg/pose_elbow.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "optimo_api/MoveItInterface.h"
#include "optimo_api/Resource.h"
#include "optimo_api/Shared.h"
#include "optimo_api/Task.h"

namespace optimo_ros
{
/**
 * @brief Servos the robot according to the given joint position topic.
 */
class EEServoCallback : public roboligent::AbstractCallback
{
public:
  /**
   * @brief Construct a new servo callback object
   *
   * @param model_ The arm model
   */
  explicit EEServoCallback(
    roboligent::Model & model_, optimo::TaskQueueObject & task_,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_);

  /**
   * @brief Get the torque calculated using model.
   *
   * @param torque_ Reference arm torque.
   */
  void calculate_torque(std::vector<int> & torque_) override;

  /**
   * @brief Used to asynchronously subscribe to a joint position topic.
   *
   */
  void create_subscription();

  /**
   * @brief Reads joint position.
   *
   * @param msg
   */
  void joint_cb(const sensor_msgs::msg::JointState & msg);

  /**
   * @brief Reads cartesian position.
   *
   * @param msg
   */
  void cartesian_cb(const optimo_msgs::msg::PoseElbow & msg);

  /**
   * @brief Get the TCP pose, needed cause get_ee_pose() returns J6 center instead.
   */
  std::vector<double> get_tcp_pose();

private:
  /**
   * @brief Converts the last cartesian pose received into a usuable servo point.
   *
   */
  void convert_pose_to_servo_point();

  roboligent::TrajectoryGoal goal;
  roboligent::Model & model;
  std::unique_ptr<roboligent::Trajectory> trajectory;
  optimo::TaskQueueObject & task;
  std::vector<double> servo_point;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
  rclcpp::Subscription<optimo_msgs::msg::PoseElbow>::SharedPtr cartesian_sub;
  optimo_msgs::msg::PoseElbow last_target_pose;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::future<void> sub_future;

  rclcpp::Time last_unique_msg_time;
  int total_messages{0};

  static std::mutex trajectory_mutex_;  // Declaration
};
}  // namespace optimo_ros
#endif
