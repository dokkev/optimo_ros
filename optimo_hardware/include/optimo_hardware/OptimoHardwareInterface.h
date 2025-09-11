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

#ifndef OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_HARDWARE_INCLUDE_OPTIMO_HARDWARE_OPTIMO_HARDWARE_INTERFACE_H_
#define OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_HARDWARE_INCLUDE_OPTIMO_HARDWARE_OPTIMO_HARDWARE_INTERFACE_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <rl/communication/EtherCATCommunication.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/set_bool.hpp>

using hardware_interface::return_type;

namespace optimo_ros
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Optimo's hardware interface, under the ROS control framework.
 *
 */
class OptimoHardwareInterface : public hardware_interface::SystemInterface
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
   * @brief Calls EtherCATCommunication::start(), which as of this writing takes ~20 seconds to
   * start and configure the robot.
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
   * @brief Enables the robot.
   *
   *  @details This method is not currently accessible, see
   * https://robotics.stackexchange.com/questions/104924/about-lifecycle-and-how-to-call-on-deactivate-in-hardware-interface-of-ro.
   * Use the optimo_hw_iface_server/toggle_enable service to enable/disable the robot.
   *
   * @return CallbackReturn
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Disables the robot, engaging the brakes.
   *
   * @details This method is not currently accessible, see
   * https://robotics.stackexchange.com/questions/104924/about-lifecycle-and-how-to-call-on-deactivate-in-hardware-interface-of-ro.
   * Use the optimo_hw_iface_server/toggle_enable service to enable/disable the robot.
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
   * @brief Exports the command interface, which is joint effort.
   *
   * @return std::vector<hardware_interface::CommandInterface>
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Reads joint data using EtherCATCommunication::read() and copies the information into
   * the state interface.
   *
   * @return return_type OK
   */
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  /**
   * @brief Writes the torque command from the command interface. Also includes a check to enable
   * the robot after a non-zero torque command is given.
   *
   * @return return_type OK
   */
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  /**
   * @brief Service callback for enabling/disabling the robot. If the service fails, the response
   * message will contain the reason why.
   *
   *
   * @param request
   * @param response
   */
  void srv_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Checks if torque command is non-zero. Should also check if the controller is reporting
   * a good status.
   *
   * @return true if the torque command is non-zero.
   */
  bool check_for_controller();

  std::vector<double> hw_eff_cmd;
  double model_safety_error;
  double communication_enabled;
  std::vector<double> hw_pos;
  std::vector<double> hw_vel;
  std::vector<double> hw_eff;

  std::string robot_index;
  bool should_enable;
  std::shared_ptr<roboligent::EtherCATCommunication> communication;
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv;
  std::thread srv_thread;
};

}  // namespace optimo_ros

#endif  // OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_HARDWARE_INCLUDE_OPTIMO_HARDWARE_OPTIMO_HARDWARE_INTERFACE_H_
