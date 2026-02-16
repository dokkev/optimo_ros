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

#include <csignal>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class ShutdownHandler : public rclcpp::Node
{
public:
  ShutdownHandler()
  : Node("shutdown_handler")
  {
    // Create service client for stop_cb
    stop_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/optimo/optimo_effort_controller/stop_cb");

    RCLCPP_INFO(this->get_logger(), "Shutdown handler initialized. Press Ctrl+C to stop servo.");
  }

  void call_stop_service()
  {
    RCLCPP_INFO(this->get_logger(), "Shutdown detected, calling stop service...");

    // Wait for service to be available (with short timeout)
    if (!stop_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Stop service not available");
      return;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service synchronously
    auto future = stop_client_->async_send_request(request);

    // Wait for response with timeout
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
        std::chrono::seconds(2)) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Stop service called successfully");
      } else {
        RCLCPP_WARN(this->get_logger(), "Stop service returned failure: %s",
                    response->message.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Stop service call timed out");
    }
  }

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
};

// Global pointer for signal handler
std::shared_ptr<ShutdownHandler> g_shutdown_handler;

void signal_handler(int signum)
{
  if (g_shutdown_handler) {
    g_shutdown_handler->call_stop_service();
  }
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create shutdown handler node
  g_shutdown_handler = std::make_shared<ShutdownHandler>();

  // Register signal handler for Ctrl+C
  std::signal(SIGINT, signal_handler);

  // Spin the node
  rclcpp::spin(g_shutdown_handler);

  return 0;
}
