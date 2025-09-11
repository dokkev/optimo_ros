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

#include "optimo_api/PlayTrajCallback.h"

#include <rl/util/Logger.h>
#include <rl/util/Vector.h>

#include "optimo_api/Resource.h"
#include "optimo_api/Shared.h"

namespace optimo
{
PlayTrajCallback::PlayTrajCallback(roboligent::Model & model_, optimo::TaskQueueObject & task_)
: roboligent::AbstractCallback(model_), task(task_)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void PlayTrajCallback::calculate_torque(std::vector<int> & torque_)
{
  switch (task->get_status()) {
    case roboligent::CommandStatus::IDLE: {
      // check if task holds a trajectory goal as a part of the command
      if (!std::holds_alternative<roboligent::TrajectoryGoal>(task->command().command_data)) {
        LOG_ERROR("PlayTrajCallback: Command parameters invalid!");
        task->set_status(roboligent::CommandStatus::FAILURE);
        return;
      }
      auto & goal = std::get<roboligent::TrajectoryGoal>(task->command().command_data);

      // Use said trajectory goal
      if (goal.traj_reader.get_type() == roboligent::TrajType::JOINT)
        traj = std::make_unique<roboligent::JointTrajectory>(model);
      else  // if the type here isn't cartesian, start() will return false
        traj = std::make_unique<roboligent::CartesianTrajectory>(model);
      if (traj->start(goal)) {
        task->set_status(roboligent::CommandStatus::IN_PROGRESS);
      } else {
        task->set_status(roboligent::CommandStatus::FAILURE);
      }
      break;
    }
    case roboligent::CommandStatus::IN_PROGRESS: {
      torque_ = torque_ + traj->calculate_torque();
      if (traj->get_status() == roboligent::TrajectoryStatus::COMPLETE)
        task->set_status(roboligent::CommandStatus::SUCCESS);
      break;
    }
  }
}

}  // namespace optimo
