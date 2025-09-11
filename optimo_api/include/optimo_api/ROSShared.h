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

#ifndef OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_API_INCLUDE_OPTIMO_API_ROSSHARED_H_
#define OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_API_INCLUDE_OPTIMO_API_ROSSHARED_H_

#include "optimo_api/Shared.h"

namespace optimo_ros
{
enum class ROSExtendedCommandID : int
{
  //-------SDK Specific Command ID 0-49 ----//
  NONE = static_cast<int>(optimo::ExtendedCommandID::NONE),
  INIT = static_cast<int>(optimo::ExtendedCommandID::INIT),
  QUIT = static_cast<int>(optimo::ExtendedCommandID::QUIT),
  DISABLE = static_cast<int>(optimo::ExtendedCommandID::DISABLE),
  ENABLE = static_cast<int>(optimo::ExtendedCommandID::ENABLE),
  RESET_FAULT = static_cast<int>(optimo::ExtendedCommandID::RESET_FAULT),
  LOCK = static_cast<int>(optimo::ExtendedCommandID::LOCK),
  UNLOCK = static_cast<int>(optimo::ExtendedCommandID::UNLOCK),
  SET_IDLE_MODE = static_cast<int>(optimo::ExtendedCommandID::SET_IDLE_MODE),
  SET_POSITION_MODE = static_cast<int>(optimo::ExtendedCommandID::SET_POSITION_MODE),
  SET_CONFIGURATION_MODE = static_cast<int>(optimo::ExtendedCommandID::SET_CONFIGURATION_MODE),
  SET_TORQUE_MODE = static_cast<int>(optimo::ExtendedCommandID::SET_TORQUE_MODE),
  SET_SEA_TORQUE_MODE = static_cast<int>(optimo::ExtendedCommandID::SET_SEA_TORQUE_MODE),
  SELECT_JOINT = static_cast<int>(optimo::ExtendedCommandID::SELECT_JOINT),
  SET_JOINT_POSITION = static_cast<int>(optimo::ExtendedCommandID::SET_JOINT_POSITION),
  SET_JOINT_TORQUE = static_cast<int>(optimo::ExtendedCommandID::SET_JOINT_TORQUE),
  SET_JOINT_LOAD = static_cast<int>(optimo::ExtendedCommandID::SET_JOINT_LOAD),
  SET_JOINT_TRAJECTORY = static_cast<int>(optimo::ExtendedCommandID::SET_JOINT_TRAJECTORY),
  SET_EE_LOAD = static_cast<int>(optimo::ExtendedCommandID::SET_EE_LOAD),
  SET_EE_TRAJECTORY = static_cast<int>(optimo::ExtendedCommandID::SET_EE_TRAJECTORY),
  RESERVED_22 = static_cast<int>(optimo::ExtendedCommandID::RESERVED_22),
  RECORD_TRAJECTORY = static_cast<int>(optimo::ExtendedCommandID::RECORD_TRAJECTORY),
  PLAY_TRAJECTORY = static_cast<int>(optimo::ExtendedCommandID::PLAY_TRAJECTORY),
  CALCULATE_LOAD = static_cast<int>(optimo::ExtendedCommandID::CALCULATE_LOAD),
  SET_LOAD = static_cast<int>(optimo::ExtendedCommandID::SET_LOAD),
  SET_HOME = static_cast<int>(optimo::ExtendedCommandID::SET_HOME),
  MOVE_HOME = static_cast<int>(optimo::ExtendedCommandID::MOVE_HOME),
  RESERVED = static_cast<int>(optimo::ExtendedCommandID::RESERVED),

  //------External IDs ONLY USE >= 50 AND <99------//
  STOP_MOTION = static_cast<int>(optimo::ExtendedCommandID::STOP_MOTION),  // 50
  CALIBRATE = static_cast<int>(optimo::ExtendedCommandID::CALIBRATE),
  TEACH = static_cast<int>(optimo::ExtendedCommandID::TEACH),
  PLAY_TRAJ = static_cast<int>(optimo::ExtendedCommandID::PLAY_TRAJ),
  FREE_MOTION = static_cast<int>(optimo::ExtendedCommandID::FREE_MOTION),
  OPTIMO_COMMUNICATION_RESERVED =
    static_cast<int>(optimo::ExtendedCommandID::OPTIMO_COMMUNICATION_RESERVED),  // 99

  //-----ROS CMD IDS >=100---//
  MOVEIT = 100,
  MOVEIT_SERVO,
  SERVO,
  SERVO_FB,

};

}  // namespace optimo_ros

#endif
