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

#ifndef OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_HARDWARE_INCLUDE_OPTIMO_HARDWARE_GRIPPER_HARDWARE_INTERFACE_H_
#define OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_HARDWARE_INCLUDE_OPTIMO_HARDWARE_GRIPPER_HARDWARE_INTERFACE_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <rl/communication/PGCANFDCommunication.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <optimo_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

using hardware_interface::return_type;

namespace optimo_ros
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Parallel gripper hardware interface, under the ROS control framework.
 *
 */
class GripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  /**
   * @brief Initializes the communication object.
   *
   * @param info
   * @return CallbackReturn
   */
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Stops communication, shutting down the robot.
   *
   * @return CallbackReturn
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Calls PGCANFDCommunication::start()
   *
   * @return CallbackReturn
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Stops communication, shutting down the robot.
   *
   * @return CallbackReturn
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Sets the operation mode to position and enables the gripper
   *
   * @return CallbackReturn
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Disables the gripper
   *
   * @return CallbackReturn
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Exports the state interface, which is joint position, velocity, and effort.
   *
   * @return std::vector<hardware_interface::StateInterface>
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Exports the command interface, which is joint position.
   *
   * @return std::vector<hardware_interface::CommandInterface>
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Reads joint data using PGCANFDCommunication::read() and copies the information into
   * the state interface.
   *
   *
   * @return return_type OK
   */
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  /**
   * @brief Writes the torque command from the command interface.
   *
   * @return return_type OK
   */
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  /**
   * @brief Set the operation mode using SetMode service.
   *
   * @param request
   * @param response
   */
  void set_operation_mode(
    const std::shared_ptr<optimo_msgs::srv::SetMode::Request> request,
    std::shared_ptr<optimo_msgs::srv::SetMode::Response> response);

  double hw_pos_cmd;
  double hw_effort_cmd;
  double hw_pos;
  double hw_vel;
  double hw_eff;

  std::shared_ptr<roboligent::PGCANFDCommunication> communication;
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Service<optimo_msgs::srv::SetMode>::SharedPtr srv;
  std::thread srv_thread;
};

}  // namespace optimo_ros

#endif  // OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_HARDWARE_INCLUDE_OPTIMO_HARDWARE_OPTIMO_HARDWARE_INTERFACE_H_
