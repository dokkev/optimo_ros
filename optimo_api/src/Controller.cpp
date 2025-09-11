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

#include "optimo_api/Controller.h"

#include <chrono>
#include <ctime>
#include <iomanip>

#include <rl/util/Logger.h>
#include <rl/util/SystemCaller.h>
#include <rl/util/Vector.h>

#include "optimo_api/Shared.h"

namespace optimo
{
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////Controller//////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

Controller::Controller(std::shared_ptr<roboligent::EtherCATCommunication> & communication_)
: AbstractController(),
  model(std::make_shared<roboligent::RobotConfiguration>(
    communication_->get_configuration_file(), communication_->get_configuration_group())),
  callback_commander(std::make_shared<CallbackCommander>(
    model, communication_->get_data(), current_task, communication_->get_configuration_file())),
  torque_command(std::vector<int>(communication_->get_data().arm_dof, 0)),
  task_queue(),
  communication(communication_)

{
  if (!communication)
    throw std::runtime_error("Controller::Controller: Not EtherCATCommunication.");
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::end_controller()
{
  AbstractController::end_controller();
  communication->stop();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::start_controller()
{
  if (!communication->start()) {
    LOG_ERROR("Controller::start_controller: Communication starting was unsuccessful.");
    sleep(1);  // cleanup logs to print out
    throw std::runtime_error(
      "Controller::start_controller: Communication starting was unsuccessful.");
  }
  AbstractController::start_controller();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::update()
{
  // Read Communication
  communication->read();

  // Update Robot State
  update_state();

  // Update Task
  if (is_task_configured()) update_task();

  std::vector<double> g_torque_command;

  // Get Torque through CallBackCommander
  if (callback_commander->command_callback(torque_command, g_torque_command)) {  // Set Torque

    // Update Model
    model.update(
      communication->get_data().rx_data.arm_position,
      communication->get_data().rx_data.arm_velocity,
      communication->get_data().rx_data.arm_acceleration, torque_command);

    communication->set_tx_arm_target_sea_torque(torque_command);
  }
  // Write Communication
  communication->write();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::update_state()
{
  // if robot goes from enabled to disabled, stop any callbacks,clear queue
  static bool prev_enabled = false;
  if (communication->get_data().optimo_status != roboligent::OptimoStatus::ENABLED) {
    if (prev_enabled) {
      roboligent::RobotCommand cmd;
      cmd.id = (int)optimo::ExtendedCommandID::STOP_MOTION;
      preempt_task(cmd);
      prev_enabled = false;
    }
  } else {
    prev_enabled = true;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool Controller::is_task_configured()
{
  // Checking if task is in progress and resetting if it is not.
  if (current_task) {
    if (current_task->is_timeout()) current_task->set_status(roboligent::CommandStatus::FAILURE);
    if (current_task->is_completed())
      complete_task();
    else
      return false;
  }

  // If there is no task Return
  if (task_queue.empty()) return false;

  // Get Next Tasks
  current_task = task_queue.front();
  // Initialize Next Task
  init_task();
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void Controller::update_task()
{
  // TODO: simplify this switch statement for the callbacks
  switch (current_task->command().id) {
    case static_cast<int>(optimo::ExtendedCommandID::SET_SEA_TORQUE_MODE):
      LOG_INFO("Controller::update_task: Setting SEA Torque.");
      current_task->start_task();
      current_task->set_status(
        communication->set_sea_torque_mode() ? roboligent::CommandStatus::SUCCESS
                                             : roboligent::CommandStatus::FAILURE);
      return;
    case static_cast<int>(optimo::ExtendedCommandID::ENABLE):
      LOG_INFO("Controller::update_task: Enabling Robot.");
      current_task->start_task();
      current_task->set_status(
        communication->enable() ? roboligent::CommandStatus::SUCCESS
                                : roboligent::CommandStatus::FAILURE);
      return;
    case static_cast<int>(optimo::ExtendedCommandID::DISABLE):
      LOG_INFO("Controller::update_task: Disabling Robot.");
      current_task->start_task();
      current_task->set_status(
        communication->disable() ? roboligent::CommandStatus::SUCCESS
                                 : roboligent::CommandStatus::FAILURE);
      return;
    case static_cast<int>(optimo::ExtendedCommandID::CALIBRATE):
      LOG_INFO("Controller::update_task: Starting Task Calibrate.");
      current_task->start_task();
      return;

    case static_cast<int>(optimo::ExtendedCommandID::RECORD_CALIBRATION):
      LOG_INFO("Controller::update_task: Starting Task Record Calibrate.");
      current_task->start_task();
      return;
    case static_cast<int>(optimo::ExtendedCommandID::PLAY_TRAJ):
      LOG_INFO("Controller::update_task: Starting to play a trajectory.");
      current_task->start_task();
      return;
    case static_cast<int>(optimo::ExtendedCommandID::TEACH):
      LOG_INFO("Controller::update_task: Starting to teach a trajectory.");
      current_task->start_task();
      return;
    case static_cast<int>(optimo::ExtendedCommandID::MOVE_HOME):
      LOG_INFO("Controller::update_task: Starting to move back to home position.");
      current_task->start_task();
      return;
    case static_cast<int>(optimo::ExtendedCommandID::FREE_MOTION):
      LOG_INFO("Controller::update_task: Starting Free MOtion");
      current_task->start_task();
      return;
    default:
      LOG_ERROR(
        "Controller::update_task: Unknown CommandID " +
        current_task->command().id);  // current_task->command().id
      current_task->set_status(roboligent::CommandStatus::FAILURE);
      complete_task();
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::init_task()
{
  switch (current_task->command().id) {
    case static_cast<int>(optimo::ExtendedCommandID::ENABLE): {
      model.reset_fallback();
      break;
    }
    case static_cast<int>(optimo::ExtendedCommandID::CALIBRATE): {
      if (!std::holds_alternative<roboligent::TrajectoryGoal>(current_task->command().command_data))
        current_task->command().command_data = roboligent::TrajectoryGoal();
      break;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

const roboligent::OptimoCommunication::OptimoData & Controller::get_data() const
{
  return communication->get_data();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

const roboligent::Model & Controller::get_model() const { return model; }

///////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::command_robot(optimo::TaskQueueObject task_) { task_queue.push(task_); }

///////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::toggle_brakes(bool turn_on) { communication->toggle_brakes(turn_on); }

///////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::complete_task()
{
  if (current_task->get_status() == roboligent::CommandStatus::FAILURE)
    LOG_INFO("Controller::complete_task: Task failed, moving to idle callback.");
  else
    LOG_INFO("Controller::complete_task: Task succeeded, moving to idle callback.");

  if (!task_queue.empty()) task_queue.pop();
  current_task = nullptr;

  // Ensure fallback doesnt jump and all safeties are at default values
  // This should probably become a helper in model
  model.reset_fallback();
  model.enable_fallback_defaults(true);
  model.enable_fallback_dragging(true);
  model.enable_elbow_fallback_dragging(true);

  model.enable_j1j7_sg_compensation(false);
  model.enable_j2_sg_compensation(false);
  model.enable_j4_sg_compensation(false);
  model.enable_j6_sg_compensation(false);
  model.set_angle_j6_sg_compensation(0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool Controller::flash_yaml()
{
  LOG_INFO("Controller::flash_yaml: Flashing YAML");

  //-----------------------------Setting Up File Names---------------------------//

  // Input File Names
  std::string input_file_old_name = communication->get_configuration_file();
  std::string input_file_new_name = input_file_old_name;

  // Renaming New Name -- Adding Time to input_file_new_name
  // Get the current time point
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  // Convert the time point to a time_t object
  std::time_t time = std::chrono::system_clock::to_time_t(now);
  // Format the time into a string
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S-");
  std::string time_str = ss.str();
  size_t lastDotPosition = input_file_new_name.find_last_of('.');
  if (lastDotPosition != std::string::npos) {
    input_file_new_name.insert(lastDotPosition, time_str);
  } else {
    LOG_ERROR(
      "Controller::flash_yaml: Invalid Filename passed through "
      "RobotConfiguration.");
    return false;
  }

  // Output File Names
  std::string output_file_new_name = communication->get_configuration_group();
  std::string output_file_old_name = output_file_new_name;

  // Renaming -- Old Name -- ADDING "_test"
  lastDotPosition = output_file_old_name.find_last_of('.');
  if (lastDotPosition != std::string::npos) {
    output_file_old_name.insert(lastDotPosition, "_test");
  } else {
    LOG_ERROR(
      "Controller::flash_yaml: Invalid Filename passed through "
      "RobotConfiguration.");
    return false;
  }

  //-----------------------------Renaming Files---------------------------//
  SYSCALL("mv " + input_file_old_name + " " + input_file_new_name);
  SYSCALL("mv " + output_file_old_name + " " + output_file_new_name);

  //-----------------------------Pushing File---------------------------//
  if (!communication->push_yaml_file(output_file_new_name)) {
    LOG_ERROR("Controller::flash_yaml: Could not push Yaml File.");
    return false;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TaskQueueObject Controller::get_current_task() { return current_task; }

///////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::preempt_task(roboligent::RobotCommand cmd)
{
  if (cmd.id == (int)ExtendedCommandID::STOP_MOTION) {
    if (current_task) {
      // fail the current task, if it exists.
      LOG_INFO("Controller::preempt_task: Callback stopped.");
      current_task->set_status(roboligent::CommandStatus::FAILURE);
      // clear the rest of the queue
      TaskQueue empty;
      std::swap(task_queue, empty);
      complete_task();
    } else {
      LOG_INFO("Controller::preempt_task: No callback to stop.");
    }
  } else if (cmd.id == (int)ExtendedCommandID::DISABLE)
    communication->disable();
  else {
    LOG_ERROR("Controller::preempt_task: Invalid cmd id.");
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

const roboligent::EtherCATCommunication::EtherCATData & Controller::get_ethercat_data() const
{
  return communication->get_ethercat_data();
}

}  // namespace optimo
