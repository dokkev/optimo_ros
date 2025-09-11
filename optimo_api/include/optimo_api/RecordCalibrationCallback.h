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

#ifndef OPTIMO_CONTROLLER_SRC_CONTROLLER_RECORDCALIBRATIONCALLBACK_H_
#define OPTIMO_CONTROLLER_SRC_CONTROLLER_RECORDCALIBRATIONCALLBACK_H_

#include <rl/communication/OptimoCommunication.h>
#include <rl/controller/AbstractCallback.h>
#include <rl/trajectory/TrajectoryRecorder.h>

#include "optimo_api/Task.h"

namespace optimo
{
/**
 * @brief Commands just model torque for transparent motion, and allows the user to record points
 * for calibration. Points are recorded using TrajectoryRecorder.
 *
 */
class RecordCalibrationCallback : public roboligent::AbstractCallback

{
public:
  /**
   * @brief Construct a new Record Calibration Callback object
   *
   * @param model_
   * @param task_
   * @param data_
   */
  explicit RecordCalibrationCallback(
    roboligent::Model & model_, optimo::TaskQueueObject & task_,
    const roboligent::OptimoCommunication::OptimoData & data_);

  /**
   * @brief Destroy the Record Calibration Callback object
   *
   */
  virtual ~RecordCalibrationCallback() = default;

  /**
   * @brief Provides torque for free motion and has the capability for saving points.
   *
   * @details saves the joint positions in radians when data flag is set to the recorder and when
   * 7 positions are saved, it stores them in the calibration.trj file.
   *
   * @param torque_
   */
  void calculate_torque(std::vector<int> & torque_) override;

private:
  optimo::TaskQueueObject & task;
  const roboligent::OptimoCommunication::OptimoData & data;
  roboligent::TrajectoryRecorder record;
};

}  // namespace optimo

#endif
