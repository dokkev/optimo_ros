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

#ifndef OPTIMO_CONTROLLER_SRC_CONTROLLER_CALLBACKCOMMANDER_H_
#define OPTIMO_CONTROLLER_SRC_CONTROLLER_CALLBACKCOMMANDER_H_

#include <rl/controller/AbstractCallback.h>

#include "optimo_api/CalibrationCallback.h"
#include "optimo_api/FreeMotionCallback.h"
#include "optimo_api/IdleCallback.h"
#include "optimo_api/MoveHomeCallback.h"
#include "optimo_api/PlayTrajCallback.h"
#include "optimo_api/RecordCalibrationCallback.h"
#include "optimo_api/TeachCallback.h"

namespace optimo
{

/**
 * @brief Callback Commander is a joint data structure that contains different types of callback
 * objects. Depending on the command in the current task, this class uses the different
 * callback object to calculate torque (Using Model, Interface and Task).
 *
 */
class CallbackCommander
{
public:
  /**
   * @brief Construct a new Callback Commander object. Pass in Reference to model_, g_model_,
   * interface_, configuration_ and TaskQueueObject_  objects.
   *
   * @param model_
   * @param g_model_
   * @param interface_
   * @param task_
   * @param configuration_
   */
  explicit CallbackCommander(
    roboligent::Model & model_, const roboligent::OptimoCommunication::OptimoData & data_,
    TaskQueueObject & task_, const std::string & configuration_file_);

  /**
   * @brief Default Destructor
   */
  virtual ~CallbackCommander() = default;

  /**
   * @brief Calls arm_cb with the torque_ param, calls attachment_cb with the g_torque_ param
   *
   * @param torque_ Reference arm torque
   * @param g_torque_ Reference gripper torque
   *
   * @return true
   * @return false if the task status was set to FAILURE, indicating that the callback went wrong
   * somehow.
   */
  bool command_callback(std::vector<int> & torque_, std::vector<double> & g_torque_);

protected:
  /**
   * @brief Checks the cmd type in the current task, and uses the corresponding object to call
   * their callback. If no task exists, runs idle callback.
   *
   * @param torque_ Reference arm torque    JogCallback jog_cb;

   */
  virtual void arm_cb(std::vector<int> & torque_);

  TaskQueueObject & task;

private:
  IdleCallback idle_cb;
  FreeMotionCallback free_cb;
  CalibrationCallback calibration_cb;
  PlayTrajCallback play_traj_cb;
  TeachCallback teach_cb;
  MoveHomeCallback move_home_cb;
  RecordCalibrationCallback rec_cali_cb;
};
}  // namespace optimo
#endif
