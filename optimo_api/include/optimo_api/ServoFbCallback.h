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

#ifndef OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_API_INCLUDE_OPTIMO_API_SERVOFBCALLBACK_H_
#define OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_API_INCLUDE_OPTIMO_API_SERVOFBCALLBACK_H_

#include <future>

#include <rl/controller/AbstractCallback.h>
#include <rl/trajectory/JointTrajectory.h>
#include <rl/util/Logger.h>
#include <rl/util/Timer.h>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "optimo_api/MoveItInterface.h"
#include "optimo_api/Task.h"

namespace optimo_ros
{
/**
 * @brief Applies a fraction of the force to the robot, given by the given effort subscription.
 * Typically used paired with ServoCallback, with this running on the master.
 */
class ServoFbCallback : public roboligent::AbstractCallback
{
public:
  /**
   * @brief Construct a new servo callback object
   *
   * @param model_ The arm model
   */
  explicit ServoFbCallback(
    roboligent::Model & model_, optimo::TaskQueueObject & task_,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_);

  /**
   * @brief Get the torque calculated using model.
   *
   * @param torque_ Reference arm torque.
   */
  void calculate_torque(std::vector<int> & torque_) override;

  /**
   * @brief Used to asynchronously subscribe to a joint torque topic.
   *
   */
  void create_subscription();

  /**
   * @brief Reads joint effort.
   *
   * @param msg
   */
  void topic_callback(const std_msgs::msg::Float64MultiArray & msg);

private:
  optimo::TaskQueueObject & task;
  std::vector<int> fb_torque;
  std::vector<roboligent::LowPass<double>> fb_filt;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub;
  std::future<void> sub_future;
};
}  // namespace optimo_ros
#endif
