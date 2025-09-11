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

#ifndef OPTIMO_CONTROLLER_SRC_CONTROLLER_CALIBRATIONCALLBACK_H_
#define OPTIMO_CONTROLLER_SRC_CONTROLLER_CALIBRATIONCALLBACK_H_

#include <rl/common/RobotConfiguration.h>
#include <rl/communication/OptimoCommunication.h>
#include <rl/controller/AbstractCallback.h>
#include <rl/trajectory/JointTrajectory.h>
#include <rl/trajectory/TrajectoryGenerator.h>
#include <rl/trajectory/TrajectoryReader.h>
#include <rl/util/Variable.h>

#include "optimo_api/Shared.h"
#include "optimo_api/Task.h"

namespace optimo
{
/**
 * @brief Calibration Callback is used to perform the calibration algorithm. The purpose of the
 * algorithm is to calibrate the spring encoders of the Optimo by measuring and and adjusting
 * encoder offsets in each joint.
 *
 * Algorithm Details:
 *
 * The constructor accepts Model, OptimoData, TaskQueueObject and the name of the configuration
 * file. Model and roboligent::OptimoCommunication::OptimoData::OptimoParameters is used to
 * calculate torques and TaskQueueObject is used to manage tasks.
 *
 * The calculate_torque function is responsible for obtaining torque values from the robot's spring
 * encoders during the calibration process.
 *
 *  - Progress Update: It updates the progress of the calibration task to indicate how far along it
 * is.
 *  - Task Status Handling: The function checks the current status of the calibration task and
 * manages different states. If the task status is IDLE, it initializes the calibration process and
 * sets the trajectory goal based on the task's parameters. If the task status is IN_PROGRESS, it
 * proceeds with the calibration process. It has three main states. MOVE_TO_NEXT: In this state, it
 * checks if the trajectory is complete. If so, it transitions to the MEASURE state. MEASURE: In
 * this state, it measures the spring torque and calculates new offsets for the spring encoders
 * based on the reference and actual torque values. It then updates the calibrated_spr_offset
 * vector. If more calibration poses are available, it transitions back to MOVE_TO_NEXT with the
 * next pose. Otherwise, it transitions to MOVE_TO_REST. MOVE_TO_REST: In this state, it checks if
 * the trajectory is complete. If the trajectory is complete, the calibration process is finalized,
 * and the task's status is set to "SUCCESS" if calibration offset values are written successfully
 * to a YAML file using the robot configuration provided.
 *
 */
class CalibrationCallback : public roboligent::AbstractCallback
{
public:
  /**
   * @brief Construct a new Calibration Callback object. It takes in Model, OptimoData,
   * TaskQueueObject, Configuration File's name and initializes the local data of the class.
   * Gravity Compensation Safety is turned off since we need to use it to calculate difference
   * between reference torque and measured torque.
   *
   * @param model_
   * @param data_
   * @param task_
   * @param configuration_file_
   */
  explicit CalibrationCallback(
    roboligent::Model & model_, const roboligent::OptimoCommunication::OptimoData & data_,
    optimo::TaskQueueObject & task_, const std::string & configuration_file_);

  /**
   * @brief Virtual Destructor
   *
   */
  virtual ~CalibrationCallback() = default;

  /**
   * @brief Populates the reference torque depending on the state of the calibration.
   *
   * @details It also handles calculating spring encoder offsets and setting and executing
   * trajectories for the calibration process. If there is a calibration file present
   * with custom points, it will use the custom points otherwise it uses default calibration
   * points from CalibrationConstants.h
   *
   *  - It updates the progress of the calibration in Task.
   *  - It calculates the torque reference and torque values for each joint.
   *  - The function then checks the current state of task and handles different states
   * accordingly.
   *  - If the state is IDLE, it initializes the calibration process and sets the
   * trajectory goal.
   *  - If the state is IN_PROGRESS, it manages the calibration process.
   *  - In MOVE_TO_NEXT state, it checks if the trajectory is complete and transitions to MEASURE
   * state.
   *  - In the MEASURE state it calculates the new offsets for the spring encoders. It updates the
   * calibrated_spr_offset vector with new offsets. If there are more calibration poses to visit
   * it moves to MOVE_TO_NEXT state with the next pose. Otherwise it Transitions to MOVE_TO_REST
   * state.
   *  - In MOVE_TO_REST state, it checks if the trajectory is complete, finishes the calibration
   * and adjusts the task accordingly.
   *
   * @logs
   *  - LOG_ERROR on timeout on the Task.
   *  - LOG_INFO on moving to new poses.
   *  - LOG_INFO on completion of calibration.
   *  - LOG_INFO on MOVE_REST completion.
   *  - LOG_INFO with old and new spring encoder offsets
   *  - LOG_ERROR on failure to calibrate due to extreme offset detection. Requires Manual Tuning.
   *
   * @param torque_ reference arm torque.
   */
  void calculate_torque(std::vector<int> & torque_) override;

private:
  /**
   * @brief This function calculates corrected spring encoder offsets based on joint constants,
   * reference torque, and measured torque values.
   *
   * @param constants Interface Constants
   * @param ref_torque Reference Torque
   * @param measured_torque Measured Torque
   * @return std::vector<int> spring encoder offsets
   */
  std::vector<int> calculate_calibration_offset(
    const std::vector<int> & ref_torque, const std::vector<int> & measured_torque);

  /**
   * @brief This function is responsible for writing the calibration results to a YAML file.
   *
   * @details It opens an input and an output file stream, reads the input YAML file, searches for
   * lines containing "spr_enc_offset," and replaces the values with the calibrated offsets for
   * each joint. The modified content is then written to the output YAML file. The input file is
   * determined using configuration and the output file has an extention "_test" after it.
   *
   * Input: "complete/path/to/config.yml"
   *
   * Output "complete/path/to/config_test.yml"
   *
   * @logs
   *  - LOG_ERROR on failure to open input or output file.
   *  - LOG_ERROR on not being able to set output filename due to errors in filename received from
   * RobotConfiguration.
   *
   * @return true on success
   * @return false on failure
   */
  bool write_calibration();

  const roboligent::OptimoCommunication::OptimoData & data;
  optimo::TaskQueueObject & task;
  std::vector<int> traj_trq;
  enum class CalibrateState : int
  {
    MOVE_TO_NEXT,
    MEASURE,
    MOVE_TO_REST
  } state;
  int pose_num;
  roboligent::Timer calibrate_timer;
  std::array<roboligent::AverageVariable<int>, 7> calibrated_spr_offset;
  roboligent::JointTrajectory trajectory;
  roboligent::TrajectoryGoal goal;
  const std::string & configuration_file;
  roboligent::TrajectoryReader reader;
  bool use_default_points;
};
}  // namespace optimo
#endif
