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

#include "optimo_api/CallbackCommander.h"

#include <rl/util/Logger.h>

namespace optimo
{
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////CallbackCommander///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackCommander::CallbackCommander(
  roboligent::Model & model_, const roboligent::OptimoCommunication::OptimoData & data_,
  TaskQueueObject & task_, const std::string & configuration_file_)
: task(task_),
  idle_cb(model_),
  free_cb(model_),
  calibration_cb(model_, data_, task_, configuration_file_),
  play_traj_cb(model_, task_),
  teach_cb(model_, task_),
  move_home_cb(model_, task_),
  rec_cali_cb(model_, task_, data_)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool CallbackCommander::command_callback(
  std::vector<int> & torque_, std::vector<double> & g_torque_)
{
  arm_cb(torque_);

  if (task && task->get_status() == roboligent::CommandStatus::FAILURE) return false;
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CallbackCommander::arm_cb(std::vector<int> & torque_)
{
  switch (task ? task->command().id : static_cast<int>(optimo::ExtendedCommandID::NONE)) {
    case static_cast<int>(optimo::ExtendedCommandID::CALIBRATE):
      calibration_cb.get_torque(torque_);
      break;

    case static_cast<int>(optimo::ExtendedCommandID::RECORD_CALIBRATION):
      rec_cali_cb.get_torque(torque_);
      break;
    case static_cast<int>(optimo::ExtendedCommandID::FREE_MOTION):
      free_cb.get_torque(torque_);
      break;
    case static_cast<int>(optimo::ExtendedCommandID::PLAY_TRAJ):
      play_traj_cb.get_torque(torque_);
      break;

    case static_cast<int>(optimo::ExtendedCommandID::TEACH):
      teach_cb.get_torque(torque_);
      break;
    case static_cast<int>(optimo::ExtendedCommandID::MOVE_HOME):
      move_home_cb.get_torque(torque_);
      break;
    case static_cast<int>(optimo::ExtendedCommandID::NONE):
    default:
      idle_cb.get_torque(torque_);
      break;
  }
}

}  // namespace optimo
