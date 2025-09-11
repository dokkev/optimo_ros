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

#include "optimo_api/MoveHomeCallback.h"

#include <rl/trajectory/TrajectoryGenerator.h>
#include <rl/util/Vector.h>

#include "optimo_api/CalibrationConstants.h"
#include "optimo_api/Shared.h"

namespace optimo
{
MoveHomeCallback::MoveHomeCallback(roboligent::Model & model_, optimo::TaskQueueObject & task_)
: roboligent::AbstractCallback(model_), task(task_), trajectory(model_)

{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MoveHomeCallback::calculate_torque(std::vector<int> & torque_)
{
  switch (task->get_status()) {
    case roboligent::CommandStatus::IDLE: {
      // Actually moves to given position if valid, home otherwise
      if (std::holds_alternative<std::vector<double>>(task->command().command_data)) {
        auto & vec = std::get<std::vector<double>>(task->command().command_data);
        if (vec.size() == 7) {
          goal.traj_reader = roboligent::generate_joint_trajectory(model.get_position(), vec);
        } else
          goal.traj_reader =
            roboligent::generate_joint_trajectory(model.get_position(), model.get_home_position());
      } else
        goal.traj_reader =
          roboligent::generate_joint_trajectory(model.get_position(), model.get_home_position());

      // Reset trajectory
      trajectory.stop();
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

}  // namespace optimo
