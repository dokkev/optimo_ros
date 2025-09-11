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

#ifndef OPTIMO_CONTROLLER_SRC_CONTROLLER_TEACHCALLBACK_H_
#define OPTIMO_CONTROLLER_SRC_CONTROLLER_TEACHCALLBACK_H_

#include <rl/controller/AbstractCallback.h>
#include <rl/trajectory/TrajectoryRecorder.h>
#include <rl/util/Logger.h>
#include <rl/util/Timer.h>

#include "optimo_api/Task.h"

namespace optimo
{
/**
 * @brief Upon starting this callback, the joint positions will be recorded to a trajectory file
 * until the callback is ended using the task object.
 */
class TeachCallback : public roboligent::AbstractCallback
{
public:
  /**
   * @brief Construct a new Teach Callback object
   *
   * @param model_ The arm model
   * @param task_ The task, with the following data fields.
   *  - task_->command().data[0] = 1 to finish recording the trajectory.
   *
   * Stopping the callback through stop motion or some other means will discard the recorded
   * trajectory.
   */
  explicit TeachCallback(roboligent::Model & model_, optimo::TaskQueueObject & task_);

  /**
   * @brief Destroy the Teach Callback object
   *
   */
  virtual ~TeachCallback() = default;

  /**
   * @brief Get the torque calculated using model.
   *
   * @param torque_ Reference arm torque.
   */
  void calculate_torque(std::vector<int> & torque_) override;

private:
  optimo::TaskQueueObject & task;
  roboligent::TrajectoryRecorder record;
  roboligent::Timer record_timer;
};
}  // namespace optimo
#endif  // OPTIMO_CONTROLLER_SRC_CONTROLLER_TEACHCALLBACK_H_
