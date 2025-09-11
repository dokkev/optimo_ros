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

#ifndef OPTIMO_CONTROLLER_SRC_CONTROLLER_JOGCALLBACK_H_
#define OPTIMO_CONTROLLER_SRC_CONTROLLER_JOGCALLBACK_H_

#include <rl/controller/AbstractCallback.h>

#include "optimo_api/Task.h"

namespace optimo
{
/**
 * @brief Jogging is a teleoperation mode where the user can move the robot in steady direction. For
 * now, the arm stays idle, while is the gripper can be opened and closed via changes in task.
 */
class JogCallback : public roboligent::AbstractCallback
{
public:
  /**
   * @brief Construct a new Jog Callback object
   *
   * @param model_ The arm model
   */
  explicit JogCallback(roboligent::Model & model_, optimo::TaskQueueObject & task_);

  /**
   * @brief Destroy the Jog Callback object
   *
   */
  virtual ~JogCallback() = default;

  /**
   * @brief Get the torque calculated using model.
   *
   * @param torque_ Reference arm torque.
   */
  void calculate_torque(std::vector<int> & torque_) override;

private:
  optimo::TaskQueueObject & task;
};
}  // namespace optimo
#endif  // OPTIMO_CONTROLLER_SRC_CONTROLLER_JOGCALLBACK_H_
