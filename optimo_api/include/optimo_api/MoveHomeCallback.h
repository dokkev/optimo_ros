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

#ifndef OPTIMO_CONTROLLER_SRC_CONTROLLER_MOVEHOMECALLBACK_H_
#define OPTIMO_CONTROLLER_SRC_CONTROLLER_MOVEHOMECALLBACK_H_

#include <rl/controller/AbstractCallback.h>
#include <rl/trajectory/JointTrajectory.h>

#include "optimo_api/Task.h"

namespace optimo
{

/**
 * @brief Moves the robot to the home position using a joint trajectory.
 */
class MoveHomeCallback : public roboligent::AbstractCallback
{
public:
  /**
   * @brief Construct a new MoveHome Callback object
   *
   * @param model_ The arm model
   */
  explicit MoveHomeCallback(roboligent::Model & model_, optimo::TaskQueueObject & task_);

  /**
   * @brief Destroy the MoveHome Callback object
   *
   */
  virtual ~MoveHomeCallback() = default;

  /**
   * @brief Get the torque calculated using model.
   *
   * @param torque_ Reference arm torque.
   */
  void calculate_torque(std::vector<int> & torque_) override;

private:
  optimo::TaskQueueObject & task;
  roboligent::TrajectoryGoal goal{};
  roboligent::JointTrajectory trajectory;
};
}  // namespace optimo
#endif  // OPTIMO_CONTROLLER_SRC_CONTROLLER_MOVEHOMECALLBACK_H_
