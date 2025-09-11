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

#include "optimo_api/ServoFbCallback.h"

#include <rl/model/ModelTypes.h>
#include <rl/util/Vector.h>

#include "optimo_api/Resource.h"
#include "optimo_api/Shared.h"

namespace optimo_ros
{
ServoFbCallback::ServoFbCallback(
  roboligent::Model & model_, optimo::TaskQueueObject & task_,
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_)
: roboligent::AbstractCallback(model_),
  task(task_),
  fb_torque(model_.size(), 0),
  fb_filt(model_.size(), roboligent::LowPass<double>(20, 0.001)),
  node(node_)
{
  sub_future = std::future<void>();
  // create_subscription();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void ServoFbCallback::calculate_torque(std::vector<int> & torque_)
{
  RCLCPP_INFO(node->get_logger(), "ServoFbCallback::calculate_torque() status: %d", static_cast<int>(task->get_status()));

  switch (task->get_status()) {
    case roboligent::CommandStatus::IDLE: {
      if (!sub_future.valid()) {
        // Start the subscription creation asynchronously
        sub_future = std::async(std::launch::async, &ServoFbCallback::create_subscription, this);
      }
      if (sub_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        if (!sub) {
          LOG_ERROR("ServoFbCallback::calculate_torque: Failed to create subscription.");
          task->set_status(roboligent::CommandStatus::FAILURE);
          return;
        }
        LOG_INFO(
          "ServoFbCallback::calculate_torque: Subscription created to " +
          std::string(sub->get_topic_name()));
        task->set_status(roboligent::CommandStatus::IN_PROGRESS);
        // reset force to zero
        for (int i = 0; i < model.size(); ++i) fb_filt[i].reset(0);
      }
      break;
    }
    case roboligent::CommandStatus::IN_PROGRESS: {
      // convert filter to normal vector, and run the filter.
      std::vector<int> fb_torque_;
      for (int i = 0; i < fb_filt.size(); ++i)
        fb_torque_.push_back(fb_filt[i](-0.25 * fb_torque[i]));
      torque_ = torque_ + fb_torque_;
      RCLCPP_INFO(node->get_logger(), "ServoFbCallback::calculate_torque: ");
      break;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// void ServoFbCallback::create_subscription()
// {
// //   auto sub_name = std::string(task->command().text.data()).length() == 0
// //                     ? "/slave/servo_fb"
// //                     : std::string(task->command().text.data());
// //   sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
// //     sub_name, 10, std::bind(&ServoFbCallback::topic_callback, this, std::placeholders::_1));


//     const std::string sub_name = "/optimo/servo_fb";
    
//     sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
//         sub_name,
//         10,
//         std::bind(&ServoFbCallback::topic_callback, this, std::placeholders::_1)
//     );

//     // print out the topic name
//   LOG_INFO("ServoFbCallback::create_subscription: Subscribed to " + sub_name);
// }

void ServoFbCallback::create_subscription()
{
    const std::string sub_name = "/optimo/servo_fb";
    
    // Add debug logging before subscription creation
    LOG_INFO("ServoFbCallback::create_subscription: Attempting to create subscription to " + sub_name);
    
    // Add node validity check
    if (!node) {
        LOG_ERROR("ServoFbCallback::create_subscription: Node is null!");
        return;
    }
    
    sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        sub_name,
        10,
        std::bind(&ServoFbCallback::topic_callback, this, std::placeholders::_1)
    );

    // Verify subscription was created
    if (!sub) {
        LOG_ERROR("ServoFbCallback::create_subscription: Failed to create subscription!");
        return;
    }

    LOG_INFO("ServoFbCallback::create_subscription: Successfully subscribed to " + sub_name);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void ServoFbCallback::topic_callback(const std_msgs::msg::Float64MultiArray & msg)
{
  // get effort information from the ros_msg
  for (int i = 0; i < model.size(); i++) {
    fb_torque[i] = msg.data[i];
  }

}

}  // namespace optimo_ros
