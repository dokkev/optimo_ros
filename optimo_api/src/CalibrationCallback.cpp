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

#include "optimo_api/CalibrationCallback.h"

#include <filesystem>
#include <fstream>

#include <rl/communication/EtherCATCommunicationTypes.h>
#include <rl/util/Logger.h>
#include <rl/util/SystemCaller.h>
#include <rl/util/Vector.h>

#include "optimo_api/CalibrationConstants.h"
#include "optimo_api/Resource.h"

namespace optimo
{
CalibrationCallback::CalibrationCallback(
  roboligent::Model & model_, const roboligent::OptimoCommunication::OptimoData & data_,
  optimo::TaskQueueObject & task_, const std::string & configuration_file_)
: AbstractCallback(
    model_,
    [&]() {
      auto safety =
        std::vector<bool>(static_cast<int>(roboligent::AbstractCallback::Safeties::COUNT), true);
      safety[static_cast<int>(roboligent::AbstractCallback::Safeties::GRAVITY_COMPENSATION)] =
        false;
      return safety;
    }()),  // turn off gravity comp
  data(data_),
  task(task_),
  traj_trq(std::vector<int>(model.size(), 0)),
  state(CalibrateState::MOVE_TO_NEXT),
  pose_num(0),
  calibrate_timer(roboligent::Timer::Milliseconds(2000)),
  trajectory(model_),
  configuration_file(configuration_file_),
  reader(),
  use_default_points(true)

{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void CalibrationCallback::calculate_torque(std::vector<int> & torque_)
{
  task->set_progress(
    use_default_points ? (double)pose_num / CALIBRATION_POSES.size()
                       : (double)pose_num / reader.get_length());
  torque_ = model.get_model_torque();

  switch (task->get_status()) {
    case roboligent::CommandStatus::IDLE: {
      std::filesystem::path filePath = optimo::RSRC_PATH() + "/calibration.trj";
      if (std::filesystem::exists(filePath)) {
        reader.reset(filePath);
        use_default_points = false;
      } else
        use_default_points = true;

      state = CalibrateState::MOVE_TO_NEXT;
      pose_num = 0;

      if (!std::holds_alternative<roboligent::TrajectoryGoal>(task->command().command_data)) {
        LOG_ERROR("CalibrationCallback:get_torque: Command parameters invalid!");
        task->set_status(roboligent::CommandStatus::FAILURE);
        return;
      }
      goal = std::get<roboligent::TrajectoryGoal>(task->command().command_data);
      if (use_default_points)
        goal.traj_reader = roboligent::generate_joint_trajectory(
          model.get_position(), roboligent::arr_to_vec(CALIBRATION_POSES[0]));
      else {
        std::vector<double> start;
        if (!reader.start(start)) {
          LOG_ERROR(
            "CalibrationCallback::calculate_torque: Could not load start of trajectory "
            "from file!");
          task->set_status(roboligent::CommandStatus::FAILURE);
          return;
        }
        goal.traj_reader = roboligent::generate_joint_trajectory(model.get_position(), start);
      }
      goal.param.stop_sensitivity = 10;

      // Reset trajectory
      trajectory.stop();
      if (trajectory.start(goal)) {
        task->set_status(roboligent::CommandStatus::IN_PROGRESS);
      } else {
        task->set_status(roboligent::CommandStatus::FAILURE);
      }
      break;
    }
    case roboligent::CommandStatus::IN_PROGRESS:
      switch (state) {
        case CalibrateState::MOVE_TO_NEXT: {
          if (trajectory.get_status() == roboligent::TrajectoryStatus::COMPLETE) {
            state = CalibrateState::MEASURE;
            calibrate_timer.start();
          }
          break;
        }
        case CalibrateState::MEASURE: {
          if (calibrate_timer.is_timeout()) {
            auto sea_torque = -1 * data.rx_data.arm_sea_torque;  // sign flip as for some reason
                                                                 // the results reported from the
                                                                 // firmware are like this...
            std::vector<int> new_offset = calculate_calibration_offset(torque_, sea_torque);
            for (int i = 0; i < 7; i++) {
              calibrated_spr_offset[i].add_value(new_offset[i]);
            }

            if (
              (use_default_points && pose_num < CALIBRATION_POSES.size() - 1) ||
              (!use_default_points && pose_num < reader.get_length() - 1)) {
              pose_num++;
              LOG_INFO(
                "CalibrationCallback::get_torque: Moving to next calibration "
                "pose");
              state = CalibrateState::MOVE_TO_NEXT;

              if (use_default_points)
                goal.traj_reader = roboligent::generate_joint_trajectory(
                  model.get_position(), roboligent::arr_to_vec(CALIBRATION_POSES[pose_num]));
              else {
                std::vector<double> point;
                if (!reader.get_point_with_index(point, pose_num)) {
                  LOG_ERROR(
                    "CalibrationCallback::calculate_torque: Could not load "
                    "point of trajectory "
                    "from file!");
                  task->set_status(roboligent::CommandStatus::FAILURE);
                  return;
                }

                goal.traj_reader =
                  roboligent::generate_joint_trajectory(model.get_position(), point);
              }

              // Reset trajectory
              if (trajectory.start(goal)) {
                task->set_status(roboligent::CommandStatus::IN_PROGRESS);
              } else {
                task->set_status(roboligent::CommandStatus::FAILURE);
              }
            } else {
              LOG_INFO(
                "CalibrationCallback::get_torque: Calibration complete, moving "
                "to rest");
              state = CalibrateState::MOVE_TO_REST;

              goal.traj_reader = roboligent::generate_joint_trajectory(
                model.get_position(), model.get_home_position());

              // Reset trajectory
              if (trajectory.start(goal)) {
                task->set_status(roboligent::CommandStatus::IN_PROGRESS);
              } else {
                task->set_status(roboligent::CommandStatus::FAILURE);
              }
            }
          }
          break;
        }
        case CalibrateState::MOVE_TO_REST: {
          if (trajectory.get_status() == roboligent::TrajectoryStatus::COMPLETE) {
            LOG_INFO("CalibrationCallback::get_torque: Move to rest complete");
            LOG_INFO(
              "CalibrationCallback::get_torque: Calibrated spring encoder "
              "offsets: " +
              std::to_string(calibrated_spr_offset[0].value()) + " " +
              std::to_string(calibrated_spr_offset[1].value()) + " " +
              std::to_string(calibrated_spr_offset[2].value()) + " " +
              std::to_string(calibrated_spr_offset[3].value()) + " " +
              std::to_string(calibrated_spr_offset[4].value()) + " " +
              std::to_string(calibrated_spr_offset[5].value()) + " " +
              std::to_string(calibrated_spr_offset[6].value()));
            LOG_INFO(
              "CalibrationCallback::get_torque: Previous spring encoder offsets: "
              "  " +
              std::to_string(data.parameters.arm_spr_enc_offset[0]) + " " +
              std::to_string(data.parameters.arm_spr_enc_offset[1]) + " " +
              std::to_string(data.parameters.arm_spr_enc_offset[2]) + " " +
              std::to_string(data.parameters.arm_spr_enc_offset[3]) + " " +
              std::to_string(data.parameters.arm_spr_enc_offset[4]) + " " +
              std::to_string(data.parameters.arm_spr_enc_offset[5]) + " " +
              std::to_string(data.parameters.arm_spr_enc_offset[6]));
            // check if calibration valid
            for (int i = 0; i < 7; i++) {
              if (
                abs(calibrated_spr_offset[i].value() - data.parameters.arm_spr_enc_offset[i]) >
                CALIBRATION_LIM[i]) {
                task->set_status(roboligent::CommandStatus::FAILURE);
                LOG_ERROR(
                  "CalibrationCallback::get_torque: Calibration failed due "
                  "to "
                  "extreme offset detected, manual "
                  "tuning and/or refurbishment may be necessary.");
                return;
              }
            }
            if (write_calibration())
              task->set_status(roboligent::CommandStatus::SUCCESS);
            else
              task->set_status(roboligent::CommandStatus::FAILURE);
          }
        }
      }
      break;
  }
  torque_ = trajectory.calculate_torque() + torque_;

  return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<int> CalibrationCallback::calculate_calibration_offset(
  const std::vector<int> & ref_torque, const std::vector<int> & measured_torque)
{
  std::vector<int> spr_enc_offset_corrected(data.arm_dof);
  for (int i = 0; i < data.arm_dof; i++) {
    spr_enc_offset_corrected[i] =
      data.parameters.arm_spr_enc_offset[i] +
      data.parameters.arm_direction[i] * data.parameters.arm_spr_mech_gear_ratio[i] *
        roboligent::EtherCATSafetyParameters::SPR_ENC_RESOLUTION *
        (ref_torque[i] - measured_torque[i]) / 360 / data.parameters.arm_spr_constant[i];
  }
  return spr_enc_offset_corrected;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool CalibrationCallback::write_calibration()
{
  // write calibration to yaml file
  // should belong in the sdk and not require a restart

  // File streams
  std::ifstream inFile;
  std::ofstream outFile;

  // Setting Up File Names
  // Input File
  const std::string INPUT_FILE = configuration_file;

  // Output File
  std::string OUTPUT_FILE = INPUT_FILE;
  // Find the last occurrence of '.'
  size_t lastDotPosition = OUTPUT_FILE.find_last_of('.');
  // Insert the new string before the last '.'
  if (lastDotPosition != std::string::npos) {
    OUTPUT_FILE.insert(lastDotPosition, "_test");
  } else {
    LOG_ERROR(
      "CalibrationCallback::write_calibration: Invalid Filename passed through "
      "RobotConfiguration.");
    return false;
  }

  // File names
  std::string inFileName = INPUT_FILE;
  std::string outFileName = OUTPUT_FILE;

  // Open the input file
  inFile.open(inFileName);

  // Check if the input file is open
  if (!inFile.is_open()) {
    task->set_status(roboligent::CommandStatus::FAILURE);
    LOG_ERROR(
      "CalibrationCallback::write_calibration: Could not read yaml file located at " + inFileName);
    return false;
  }

  // Open the output file
  outFile.open(outFileName);

  // Check if the output file is open
  if (!outFile.is_open()) {
    task->set_status(roboligent::CommandStatus::FAILURE);
    LOG_ERROR(
      "CalibrationCallback::write_calibration: Could not write to yaml file located at " +
      outFileName);
    return false;
  }

  // Search and replace
  std::string line;
  int joint_num = 0;
  while (std::getline(inFile, line)) {
    // Search for the string "spr_enc_offset:"
    size_t pos = line.find("spr_enc_offset: ");
    if (pos != std::string::npos && joint_num < 7) {
      // Replace the found string with "spr_enc_offset: 1000 #"
      line.replace(
        pos, 15,
        "spr_enc_offset: " + std::to_string(calibrated_spr_offset[joint_num].value()) + " #");
      joint_num++;
    }
    // Write the modified line to the output file
    outFile << line << std::endl;
  }
  inFile.close();
  outFile.close();

  return true;
}

}  // namespace optimo
