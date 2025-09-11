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

#include "optimo_hardware/GripperHardwareInterface.h"

#include <algorithm>
#include <string>

#include <rl/model/ModelTypes.h>
#include <rl/util/Logger.h>
#include <rl/util/Vector.h>

#include <rclcpp/utilities.hpp>

#include "optimo_api/Resource.h"

namespace optimo_ros
{
CallbackReturn GripperHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  roboligent::Logger::Override(roboligent::Logger::INFO, "./log/ros_hardware_interface.log");

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  // We therefore need to make sure to actually deactivate the communication
  // See https://github.com/ros-controls/ros2_control/issues/472
  rclcpp::on_shutdown(
    std::bind(&GripperHardwareInterface::on_shutdown, this, rclcpp_lifecycle::State()));

  auto transport_index = info.hardware_parameters.at("robot_index");
  if (transport_index.empty()) {
    LOG_ERROR("GripperHardwareInterface::on_init: No transport index was given!");
    return CallbackReturn::ERROR;
  }

  communication = std::make_shared<roboligent::PGCANFDCommunication>(
    std::vector<int>(1, std::stoi(transport_index) + 1), std::stoi(transport_index));

  hw_effort_cmd = hw_pos_cmd = hw_pos = hw_vel = hw_eff = 0;

  // Create service for setting op modes
  node = rclcpp::Node::make_shared("pg_hw_iface_server");
  srv = node->create_service<optimo_msgs::srv::SetMode>(
    "~/switch_operation_mode", std::bind(
                                 &GripperHardwareInterface::set_operation_mode, this,
                                 std::placeholders::_1, std::placeholders::_2));
  srv_thread = std::thread([this]() { rclcpp::spin(node); });

  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn GripperHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (communication) communication->stop();
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn GripperHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  communication->start();
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn GripperHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  communication->stop();
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn GripperHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  communication->set_operation_mode(roboligent::PGOPMode::POSITION);
  if (!communication->enable()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to enable gripper");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(node->get_logger(), "Activating gripper");
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn GripperHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  communication->disable();
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<hardware_interface::StateInterface> GripperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Receive the first joint name, second one is mimic
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_eff));

  return state_interfaces;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<hardware_interface::CommandInterface>
GripperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_pos_cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_effort_cmd));

  return command_interfaces;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

return_type GripperHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  communication->read();

  // TODO some unit changes may be necessary.
  auto data = communication->get_data();
  // everything halved since joint only follow the left finger
  hw_pos = data.rx_data.attachment_position[0] / 2;
  hw_vel = data.rx_data.attachment_velocity[0] / 2;
  hw_eff = data.rx_data.attachment_torque[0] / 2;

  return return_type::OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

return_type GripperHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  switch (communication->get_data().op_mode) {
    case roboligent::PGOPMode::POSITION:
      communication->set_tx_attachment_position({hw_pos_cmd});
      break;
    case roboligent::PGOPMode::TORQUE:
      communication->set_tx_attachment_torque({hw_effort_cmd});
      break;
    case roboligent::PGOPMode::DISABLED:
    default:
      break;
  }

  communication->write();

  return return_type::OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void GripperHardwareInterface::set_operation_mode(
  const std::shared_ptr<optimo_msgs::srv::SetMode::Request> request,
  std::shared_ptr<optimo_msgs::srv::SetMode::Response> response)
{
  std::string new_mode = request->mode;

  if (new_mode == "disabled") {
    if (communication->get_data().op_mode == roboligent::PGOPMode::DISABLED) {
      response->success = false;
      response->message = "Already in the requested mode.";
    } else {
      response->success = communication->disable();
      response->message =
        response->success ? "Mode switched successfully." : "Mode switching failed.";
    }
  } else if (new_mode == hardware_interface::HW_IF_EFFORT) {
    if (communication->get_data().op_mode == roboligent::PGOPMode::TORQUE) {
      response->success = false;
      response->message = "Already in the requested mode.";
    } else {
      response->success = communication->set_operation_mode(roboligent::PGOPMode::TORQUE);
      if (!communication->get_data().enabled) communication->enable();
      response->message =
        response->success ? "Mode switched successfully." : "Mode switching failed.";
    }
  } else if (new_mode == hardware_interface::HW_IF_POSITION) {
    if (communication->get_data().op_mode == roboligent::PGOPMode::POSITION) {
      response->success = false;
      response->message = "Already in the requested mode.";
    } else {
      response->success = communication->set_operation_mode(roboligent::PGOPMode::POSITION);
      if (!communication->get_data().enabled) communication->enable();
      response->message =
        response->success ? "Mode switched successfully." : "Mode switching failed.";
    }
  } else {
    response->success = false;
    response->message = "Invalid mode requested.";
    return;
  }
}

}  // namespace optimo_ros

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(optimo_ros::GripperHardwareInterface, hardware_interface::SystemInterface)
