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

#include "optimo_api/TeachCallback.h"

#include <rl/util/Vector.h>

#include "optimo_api/Shared.h"

namespace optimo
{
TeachCallback::TeachCallback(roboligent::Model & model_, optimo::TaskQueueObject & task_)
: roboligent::AbstractCallback(model_),
  task(task_),
  record(""),
  record_timer(roboligent::Timer::Seconds(60))
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TeachCallback::calculate_torque(std::vector<int> & torque_)
{
  // done because roboligent::LowPass doesn't support std::array
  static roboligent::LowPass<Eigen::Matrix<double, 7, 1>> cuff_pos_filter =
    roboligent::LowPass<Eigen::Matrix<double, 7, 1>>(
      2, roboligent::SAMPLING_TIME, Eigen::Matrix<double, 7, 1>::Zero());

  Eigen::Matrix<double, 7, 1> pos_init;
  pos_init << model.get_joint(0).position, model.get_joint(1).position, model.get_joint(2).position,
    model.get_joint(3).position, model.get_joint(4).position, model.get_joint(5).position,
    model.get_joint(6).position;
  cuff_pos_filter(pos_init);

  switch (task->get_status()) {
    case roboligent::CommandStatus::IDLE: {
      record.reset(std::string(task->command().text.data()));
      record_timer.start();
      cuff_pos_filter =
        roboligent::LowPass<Eigen::Matrix<double, 7, 1>>(2, roboligent::SAMPLING_TIME, pos_init);
      task->set_status(roboligent::CommandStatus::IN_PROGRESS);
      break;
    }
    case roboligent::CommandStatus::IN_PROGRESS:
      record.record(
        {cuff_pos_filter.value()[0], cuff_pos_filter.value()[1], cuff_pos_filter.value()[2],
         cuff_pos_filter.value()[3], cuff_pos_filter.value()[4], cuff_pos_filter.value()[5],
         cuff_pos_filter.value()[6]});
      break;
  }

  if (record_timer.is_timeout() || task->command().data[0] == 1) {
    record.store() ? task->set_status(roboligent::CommandStatus::SUCCESS)
                   : task->set_status(roboligent::CommandStatus::FAILURE);
  }
}

}  // namespace optimo
