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

#include "optimo_api/JointTrajectoryCallback.h"

#include <rl/model/ModelTypes.h>
#include <rl/util/Vector.h>

#include "optimo_api/Shared.h"

namespace optimo
{
JointTrajectoryCallback::JointTrajectoryCallback(
  roboligent::Model & model_, optimo::TaskQueueObject & task_)
: roboligent::AbstractCallback(model_),
  trajectory(model_),
  task(task_)
{ 
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void JointTrajectoryCallback::calculate_torque(std::vector<int> & torque_)
{
  switch (task->get_status()) {
    case roboligent::CommandStatus::IDLE: {
      // check if task holds a trajectory goal as a part of the command
      if (!std::holds_alternative<roboligent::TrajectoryGoal>(task->command().command_data)) {
        LOG_ERROR("JointTrajectoryCallback: Command parameters invalid!");
        task->set_status(roboligent::CommandStatus::FAILURE);
        return;
      }
      auto & goal = std::get<roboligent::TrajectoryGoal>(task->command().command_data);

      // Use said trajectory goal
      if (goal.traj_reader.get_type() != roboligent::TrajType::JOINT){
        LOG_ERROR("JointTrajectoryCallback: Command parameters invalid!");
        task->set_status(roboligent::CommandStatus::FAILURE);
        return;
      }

      if (trajectory.start(goal)) {
        task->set_status(roboligent::CommandStatus::IN_PROGRESS);
      } else {
        task->set_status(roboligent::CommandStatus::FAILURE);
      }
      break;
    }
    case roboligent::CommandStatus::IN_PROGRESS: {
      torque_ = torque_ + trajectory.calculate_torque();
      if (trajectory.get_status() == roboligent::TrajectoryStatus::COMPLETE)
        task->set_status(roboligent::CommandStatus::SUCCESS);
      break;
    }
  }
}

}  // namespace optimo_ros
