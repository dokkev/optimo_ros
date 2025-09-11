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

/**
 * @file CalibrationConstants.h
 * @author Roboligent (info@roboligent.com)
 * @date 2023-11-07
 *
 * @copyright Copyright Roboligent(c) 2024
 *
 * Dependencies:
 *
 * @copyright Copyright Boost File System 2024.
 * See accompanying file LICENSES/boost_LICENSE
 * @copyright Copyright Google Remote Protocol Buffers (gRPC) 2024.
 * See accompanying file LICENSES/gRPC_LICENSE
 * @copyright Copyright Protocol Buffers (protobuf) 2024.
 * See accompanying file LICENSES/protobuf_LICENSE
 * @copyright Copyright EtherCAT 2024.
 * See accompanying file LICENSES/EtherCAT_LICENSE
 * @copyright Copyright Gazebo 2024.
 * See accompanying file LICENSES/Gazebo_LICENSE
 * @copyright Copyright Canberra-gtk (libcanberra) 2024.
 * See accompanying file LICENSES/libcanberra_LICENSE
 * @copyright Copyright Qt5 2024.
 * See accompanying file LICENSES/Qt_LICENSE
 * @copyright Copyright ROS2 2024.
 * See accompanying file LICENSES/ROS2_LICENSE
 * @copyright Copyright Yaml Parser 2024.
 * See accompanying file LICENSES/YAML_LICENSE
 * @copyright Copyright Eigen 2024.
 * See accompanying file LICENSES/EIGEN_LICENSE
 * @copyright Copyright can-utils 2024.
 * See accompanying file LICENSES/can-utils_LICENSE
 * @copyright Copyright SocketCAN 2024.
 * See accompanying file LICENSES/SocketCAN_LICENSE
 *
 *
 */

#ifndef OPTIMO_CONTROLLER_SRC_CONTROLLER_CALIBRATIONCONSTANTS_H_
#define OPTIMO_CONTROLLER_SRC_CONTROLLER_CALIBRATIONCONSTANTS_H_

#include <rl/model/ModelConstants.h>

namespace optimo
{
const std::array<double, 7> HOME_ARM_LEFT_SITTING{
  -29 * roboligent::DEG2RAD, 112 * roboligent::DEG2RAD, 0 * roboligent::DEG2RAD,
  -76 * roboligent::DEG2RAD, -32 * roboligent::DEG2RAD, 49 * roboligent::DEG2RAD,
  106 * roboligent::DEG2RAD};
const std::array<double, 7> HOME_ARM_LEFT_SUPINE{
  -29 * roboligent::DEG2RAD, 112 * roboligent::DEG2RAD, 0 * roboligent::DEG2RAD,
  -76 * roboligent::DEG2RAD, -32 * roboligent::DEG2RAD, 49 * roboligent::DEG2RAD,
  106 * roboligent::DEG2RAD};
const std::array<double, 7> HOME_ARM_RIGHT_SITTING{
  29 * roboligent::DEG2RAD,  112 * roboligent::DEG2RAD, 0 * roboligent::DEG2RAD,
  -76 * roboligent::DEG2RAD, 32 * roboligent::DEG2RAD,  49 * roboligent::DEG2RAD,
  -106 * roboligent::DEG2RAD};
const std::array<double, 7> HOME_ARM_RIGHT_SUPINE{
  29 * roboligent::DEG2RAD,  112 * roboligent::DEG2RAD, 0 * roboligent::DEG2RAD,
  -76 * roboligent::DEG2RAD, 32 * roboligent::DEG2RAD,  49 * roboligent::DEG2RAD,
  -106 * roboligent::DEG2RAD};

const std::array<double, 7> HOME_KNEE_LEFT_SITTING{
  -33.0 * roboligent::DEG2RAD, 120.0 * roboligent::DEG2RAD, 5.9 * roboligent::DEG2RAD,
  -79.0 * roboligent::DEG2RAD, -40.0 * roboligent::DEG2RAD, -95.3 * roboligent::DEG2RAD,
  100.0 * roboligent::DEG2RAD};
const std::array<double, 7> HOME_KNEE_RIGHT_SITTING{
  33.0 * roboligent::DEG2RAD,  120.0 * roboligent::DEG2RAD, -5.9 * roboligent::DEG2RAD,
  -79.0 * roboligent::DEG2RAD, 40.0 * roboligent::DEG2RAD,  -95.3 * roboligent::DEG2RAD,
  -100.0 * roboligent::DEG2RAD};

const std::array<double, 7> HOME_LEG_LEFT_SUPINE{
  -35 * roboligent::DEG2RAD,  130 * roboligent::DEG2RAD, 50 * roboligent::DEG2RAD,
  -130 * roboligent::DEG2RAD, -70 * roboligent::DEG2RAD, -35 * roboligent::DEG2RAD,
  70 * roboligent::DEG2RAD};
const std::array<double, 7> HOME_LEG_RIGHT_SUPINE{
  35 * roboligent::DEG2RAD,   130 * roboligent::DEG2RAD, -50 * roboligent::DEG2RAD,
  -130 * roboligent::DEG2RAD, 70 * roboligent::DEG2RAD,  -35 * roboligent::DEG2RAD,
  -70 * roboligent::DEG2RAD};
const std::array<double, 7> HOME_LEG_LEFT_STANDING{
  -26 * roboligent::DEG2RAD, 80 * roboligent::DEG2RAD,  27 * roboligent::DEG2RAD,
  -80 * roboligent::DEG2RAD, -85 * roboligent::DEG2RAD, -20 * roboligent::DEG2RAD,
  52 * roboligent::DEG2RAD};
const std::array<double, 7> HOME_LEG_RIGHT_STANDING{
  26 * roboligent::DEG2RAD,  80 * roboligent::DEG2RAD, -27 * roboligent::DEG2RAD,
  -80 * roboligent::DEG2RAD, 85 * roboligent::DEG2RAD, -20 * roboligent::DEG2RAD,
  -52 * roboligent::DEG2RAD};

// maximum change in spring enc offset that can be automatically done
const int CALIBRATION_LIM[7] = {3000, 3000, 2500, 2500, 1500, 1500, 1500};
/** @brief Calbration Poses*/
const std::array<std::array<double, 7>, 7> CALIBRATION_POSES{
  HOME_ARM_LEFT_SITTING, HOME_ARM_RIGHT_SITTING, HOME_KNEE_LEFT_SITTING, HOME_KNEE_RIGHT_SITTING,
  HOME_LEG_LEFT_SUPINE,  HOME_LEG_LEFT_STANDING, HOME_LEG_RIGHT_STANDING};
}  // namespace optimo
#endif
