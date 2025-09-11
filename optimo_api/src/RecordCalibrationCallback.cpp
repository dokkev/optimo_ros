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



#include "optimo_api/RecordCalibrationCallback.h"

#include <rl/util/SystemCaller.h>

#include "optimo_api/Resource.h"

namespace optimo
{
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////RecordCalibrationCallback///////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

RecordCalibrationCallback::RecordCalibrationCallback(
  roboligent::Model & model_, optimo::TaskQueueObject & task_,
  const roboligent::OptimoCommunication::OptimoData & data_)
: roboligent::AbstractCallback(model_),
  task(task_),
  data(data_),
  record(optimo::RSRC_PATH() + "/calibration")
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void RecordCalibrationCallback::calculate_torque(std::vector<int> & torque_)
{
  switch (task->get_status()) {
    case roboligent::CommandStatus::IDLE: {
      record.reset(optimo::RSRC_PATH() + "/calibration");
      task->set_status(roboligent::CommandStatus::IN_PROGRESS);

      break;
    }
    case roboligent::CommandStatus::IN_PROGRESS: {
      if (task->command().data[0] == 1) {
        task->set_progress(task->get_progress() + ((1.0 / 7.0) * 100.0));
        // record value here
        auto pos = data.rx_data.arm_position;
        for (int i = 0; i < data.arm_dof; i++)
          pos[i] = data.rx_data.arm_position[i] * roboligent::DEG2RAD;
        record.record(pos);

        task->command().data[0] = 0;

        if (task->get_progress() >= 99) {
          // store value here
          record.store() ? task->set_status(roboligent::CommandStatus::SUCCESS)
                         : task->set_status(roboligent::CommandStatus::FAILURE);
          return;
        }
      }
      break;
    }
  }
}

}  // namespace optimo
