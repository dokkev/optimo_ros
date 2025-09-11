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

#include "optimo_hardware/OptimoHardwareInterface.h"

#include <algorithm>
#include <string>

#include <rl/model/ModelTypes.h>
#include <rl/util/Logger.h>
#include <rl/util/Vector.h>

#include <rclcpp/utilities.hpp>

#include "optimo_api/Resource.h"

namespace optimo_ros
{
CallbackReturn OptimoHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  roboligent::Logger::Override(roboligent::Logger::INFO, "./log/ros_hardware_interface.log");

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  // We therefore need to make sure to actually deactivate the communication
  // See https://github.com/ros-controls/ros2_control/issues/472
  rclcpp::on_shutdown(
    std::bind(&OptimoHardwareInterface::on_shutdown, this, rclcpp_lifecycle::State()));

  robot_index = info.hardware_parameters.at("robot_index");
  if (robot_index.empty()) {
    LOG_ERROR("OptimoHardwareInterface::on_init: No EtherCAT master index was given!");
    return CallbackReturn::ERROR;
  }

  communication = std::make_shared<roboligent::EtherCATCommunication>(
    optimo::RSRC_PATH() + "/master" + robot_index + "/OR7_config.yml", "right_arm",
    std::stoi(robot_index));

  hw_eff_cmd.resize(communication->get_data().arm_dof, 0);
  hw_pos.resize(communication->get_data().arm_dof, 0);
  hw_vel.resize(communication->get_data().arm_dof, 0);
  hw_eff.resize(communication->get_data().arm_dof, 0);
  should_enable = false;

  // Create service for setting callbacks
  node = rclcpp::Node::make_shared("optimo_hw_iface_server");
  srv = node->create_service<std_srvs::srv::SetBool>(
    "~/toggle_enable",
    std::bind(
      &OptimoHardwareInterface::srv_cb, this, std::placeholders::_1, std::placeholders::_2));
  srv_thread = std::thread([this]() { rclcpp::spin(node); });

  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn OptimoHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (communication) communication->stop();
  srv_thread.detach();  // join doesn't work for some reason
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn OptimoHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (std::stoi(robot_index) == 1) {
    LOG_INFO("Experimental wait for second arm, sleeping 10 sec");
    sleep(10);
  }
  if (!communication->start()) {
    LOG_ERROR("OptimoHardwareInterface::on_configure: Failed to start communication!");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn OptimoHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!communication->stop()) {
    LOG_ERROR("OptimoHardwareInterface::on_cleanup: Failed to stop communication!");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn OptimoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  should_enable = true;
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn OptimoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!communication->disable()) {
    LOG_ERROR("OptimoHardwareInterface::on_deactivate: Failed to disable synapticon boards!");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<hardware_interface::StateInterface> OptimoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos.at(i)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel.at(i)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_eff.at(i)));
  }
  // Add enabled flag
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("", "communication_enabled", &communication_enabled));

  return state_interfaces;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<hardware_interface::CommandInterface>
OptimoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.reserve(info_.joints.size());
  for (auto i = 0U; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_eff_cmd.at(i)));
  }
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("", "model_safety_error", &model_safety_error));

  return command_interfaces;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

return_type OptimoHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  communication->read();

  // copy pos vel eff one by one to avoid nan race condition
  auto data = communication->get_data();
  for (int i = 0; i < 7; ++i) {
    hw_pos[i] = roboligent::DEG2RAD * data.rx_data.arm_position[i];
    hw_vel[i] = roboligent::DEG2RAD * data.rx_data.arm_velocity[i];
    hw_eff[i] = roboligent::MILLI2UNIT * data.rx_data.arm_sea_torque[i];
  }
  communication_enabled = data.enabled;

  return return_type::OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

return_type OptimoHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (should_enable && !communication->get_data().enabled && check_for_controller()) {
    communication->set_sea_torque_mode();
    communication->enable();
    should_enable = false;
  }

  if (model_safety_error > 0 && communication->get_data().enabled) {
    LOG_ERROR_THROTTLE("Error detected in model, disabling hw.", 1);
    communication->disable();
  }

  std::vector<int> tx_arm_target_sea_torque;
  std::transform(
    hw_eff_cmd.begin(), hw_eff_cmd.end(), std::back_inserter(tx_arm_target_sea_torque),
    [](double val) { return roboligent::UNIT2MILLI * val; });

  communication->set_tx_arm_target_sea_torque(tx_arm_target_sea_torque);
  communication->write();

  return return_type::OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoHardwareInterface::srv_cb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (!communication->get_data().enabled) {
      should_enable = true;
      sleep(1);
      if (communication->get_data().enabled)
        response->success = true;
      else {
        response->success = false;
        response->message = "Failed to enable.";
      }
    } else {
      response->success = false;
      response->message = "Already enabled.";
    }
  } else {
    if (!communication->get_data().enabled) {
      response->success = false;
      response->message = "Already disabled.";
    } else {
      response->success = communication->disable();
      response->message = response->success ? "Disabled successfully." : "Failed to disable.";
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool OptimoHardwareInterface::check_for_controller()
{
  return std::any_of(hw_eff_cmd.begin(), hw_eff_cmd.end(), [](auto cmd) { return cmd != 0; });
}

}  // namespace optimo_ros

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(optimo_ros::OptimoHardwareInterface, hardware_interface::SystemInterface)
