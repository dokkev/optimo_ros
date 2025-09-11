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

#include "optimo_api/MoveItServoCallback.h"

#include <rl/model/ModelTypes.h>
#include <rl/util/Vector.h>

#include "optimo_api/Resource.h"
#include "optimo_api/Shared.h"

namespace optimo_ros
{
MoveItServoCallback::MoveItServoCallback(
  roboligent::Model & model_, optimo::TaskQueueObject & task_, MoveItInterface & moveit_iface_)
: roboligent::AbstractCallback(model_), trajectory(model_), task(task_), moveit_iface(moveit_iface_)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MoveItServoCallback::calculate_torque(std::vector<int> & torque_)
{
  switch (task->get_status()) {
    case roboligent::CommandStatus::IDLE: {
      if (!moveit_iface.is_valid()) {
        LOG_ERROR(
          "MoveItServoCallback::calculate_torque: Move group interface not active. "
          "Please check if the move group node was launched.");
        task->set_status(roboligent::CommandStatus::FAILURE);
        return;
      }

      // for some reason, adding the bounding boxes in MoveItInterface causes them to only to
      // be reflected in normal MoveIt, but not it servo, most likely because it must be added
      // later.
      // moveit_iface.add_bounding_box({0.01, 1.2, 1}, {0.8, 0, 1.25});
      // moveit_iface.add_bounding_box({0.01, 1.2, 1}, {-0.4, 0, 1.25});
      // moveit_iface.add_bounding_box({1.2, 0.01, 1}, {0.2, 0.6, 1.25});
      // moveit_iface.add_bounding_box({1.2, 0.01, 1}, {0.2, -0.6, 1.25});
      // cameras
      moveit_iface.add_bounding_box({.1, .3, .1}, {-.15, .28, 1.69});
      moveit_iface.add_bounding_box({.3, .1, .1}, {-.05, .29, 1.40});

      // Reset trajectory
      trajectory.stop();
      roboligent::TrajectoryGoal goal;
      goal.param.servo = true;
      goal.param.impedance = 6;
      goal.param.max_force = 4;
      goal.param.stop_sensitivity = 10;
      trajectory.start(goal);
      task->set_status(roboligent::CommandStatus::IN_PROGRESS);
      break;
    }
    case roboligent::CommandStatus::IN_PROGRESS: {
      std::vector<double> servo_point;
      if (!moveit_iface.get_servo_point(servo_point)) {
        task->set_status(roboligent::CommandStatus::FAILURE);
        LOG_ERROR(
          "MoveItServoCallback::calculate_torque: Invalid point received."
          "Please check if the move group is still alive.");
        return;
      }

      torque_ = torque_ + trajectory.calculate_servo_torque(servo_point);
    } break;
  }
}

}  // namespace optimo_ros
