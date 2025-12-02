/**
 * @file Shared.h
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

#ifndef OPTIMO_CONTROLLER_SRC_COMMON_SHARED_H_
#define OPTIMO_CONTROLLER_SRC_COMMON_SHARED_H_

#include <rl/common/RobotCommand.h>

namespace optimo
{
/**
 * @brief ExtendedCommandID is an enum extending CommandID. The Controller
 * should use the extended commands and the original commands to command robot. Additional commands
 * can be added here, but they SHOULD be >=50 & <99. This is because 0-49 are reserved
 * for the SDK. And >99 is devoted to ROS. To see what the CommandID commands do, please refer the
 * doc for COMMANDID.
 *
 * ROS RESERVED COMMANDS: >=100
 * ALLOWED COMMANDS: >=50 && <99
 * SDK RESERVED COMMANDS: <49
 *
 * NOTE: There will be no compilation error if you do not follow instructions. However run-time
 * behaviour depends on implementation.
 *
 */
enum class ExtendedCommandID : int {
    //-------SDK Specific Command ID 0-49 ----//
    NONE = static_cast<int>(roboligent::CommandID::NONE),
    INIT = static_cast<int>(roboligent::CommandID::INIT),
    QUIT = static_cast<int>(roboligent::CommandID::QUIT),
    DISABLE = static_cast<int>(roboligent::CommandID::DISABLE),
    ENABLE = static_cast<int>(roboligent::CommandID::ENABLE),
    RESET_FAULT = static_cast<int>(roboligent::CommandID::RESET_FAULT),
    LOCK = static_cast<int>(roboligent::CommandID::LOCK),
    UNLOCK = static_cast<int>(roboligent::CommandID::UNLOCK),
    SET_IDLE_MODE = static_cast<int>(roboligent::CommandID::SET_IDLE_MODE),
    SET_POSITION_MODE = static_cast<int>(roboligent::CommandID::SET_POSITION_MODE),
    SET_CONFIGURATION_MODE = static_cast<int>(roboligent::CommandID::SET_CONFIGURATION_MODE),
    SET_TORQUE_MODE = static_cast<int>(roboligent::CommandID::SET_TORQUE_MODE),
    SET_SEA_TORQUE_MODE = static_cast<int>(roboligent::CommandID::SET_SEA_TORQUE_MODE),
    SELECT_JOINT = static_cast<int>(roboligent::CommandID::SELECT_JOINT),
    SET_JOINT_POSITION = static_cast<int>(roboligent::CommandID::SET_JOINT_POSITION),
    SET_JOINT_TORQUE = static_cast<int>(roboligent::CommandID::SET_JOINT_TORQUE),
    SET_JOINT_LOAD = static_cast<int>(roboligent::CommandID::SET_JOINT_LOAD),
    SET_JOINT_TRAJECTORY = static_cast<int>(roboligent::CommandID::SET_JOINT_TRAJECTORY),
    SET_EE_LOAD = static_cast<int>(roboligent::CommandID::SET_EE_LOAD),
    SET_EE_TRAJECTORY = static_cast<int>(roboligent::CommandID::SET_EE_TRAJECTORY),
    RESERVED_22 = static_cast<int>(roboligent::CommandID::RESERVED_22),
    RECORD_TRAJECTORY = static_cast<int>(roboligent::CommandID::RECORD_TRAJECTORY),
    PLAY_TRAJECTORY = static_cast<int>(roboligent::CommandID::PLAY_TRAJECTORY),
    CALCULATE_LOAD = static_cast<int>(roboligent::CommandID::CALCULATE_LOAD),
    SET_LOAD = static_cast<int>(roboligent::CommandID::SET_LOAD),
    SET_HOME = static_cast<int>(roboligent::CommandID::SET_HOME),
    MOVE_HOME = static_cast<int>(roboligent::CommandID::MOVE_HOME),
    RESERVED = static_cast<int>(roboligent::CommandID::RESERVED),

    //------External IDs ONLY USE >= 50 AND <100------//
    STOP_MOTION,  // 50
    CALIBRATE,
    RECORD_CALIBRATION,
    TEACH,
    PLAY_TRAJ,
    FREE_MOTION,
    JOINT_TRAJECTORY,

    OPTIMO_COMMUNICATION_RESERVED = 99
};
}  // namespace optimo

#endif
