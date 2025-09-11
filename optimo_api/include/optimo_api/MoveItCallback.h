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

#ifndef OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_API_INCLUDE_OPTIMO_API_MOVEITCALLBACK_H_
#define OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_API_INCLUDE_OPTIMO_API_MOVEITCALLBACK_H_

#include <future>

#include <rl/controller/AbstractCallback.h>
#include <rl/trajectory/JointTrajectory.h>
#include <rl/trajectory/TrajectoryReader.h>
#include <rl/util/Logger.h>
#include <rl/util/Timer.h>

#include "optimo_api/MoveItInterface.h"
#include "optimo_api/Task.h"

namespace optimo_ros
{
/**
 * @brief Performs a joint trajectory, commanded using MoveIt. Make a request using MoveItInterface,
 * and once the trajectory is received, plays this trajectory, interpolating the points as needed.
 *
 */
class MoveItCallback : public roboligent::AbstractCallback
{
public:
  /**
   * @brief Construct a new MoveIt Callback object
   *
   * @param model_ The arm model
   */
  explicit MoveItCallback(
    roboligent::Model & model_, optimo::TaskQueueObject & task_, MoveItInterface & moveit_iface_);

  /**
   * @brief Destroy the MoveIt Callback object
   *
   */
  virtual ~MoveItCallback() = default;

  /**
   * @brief Get the torque calculated using model.
   *
   * @param torque_ Reference arm torque.
   */
  void calculate_torque(std::vector<int> & torque_) override;

private:
  roboligent::Timer timer;

  roboligent::TrajectoryGoal goal;
  roboligent::JointTrajectory trajectory;
  optimo::TaskQueueObject & task;
  enum class MoveItState : int
  {
    WAIT,
    FOLLOW
  } state;

  MoveItInterface & moveit_iface;
  std::future<std::vector<std::vector<double>>> result_future;
};
}  // namespace optimo_ros
#endif
