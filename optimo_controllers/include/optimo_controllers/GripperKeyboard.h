/**
 * @file GripperKeyboard.h
 * @author Roboligent (info@roboligent.com)
 * @date 2024-07-22
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
#ifndef GRIPPER_CONTROL_NODE_HPP
#define GRIPPER_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

/**
 * @brief Used to read keyboard input.
 *
 */
class KeyboardReader
{
   public:
    /**
     * @brief Construct a new Keyboard Reader object
     *
     */
    KeyboardReader();

    /**
     * @brief Reads one character from the keyboard.
     *
     * @param c
     */
    void readOne(char *c);

    /**
     * @brief Shutdown reading.
     *
     */
    void shutdown();

   private:
    int kfd;
    struct termios cooked;
};

/**
 * @brief Publishes gripper and task status states.
 *
 */
class GripperControlNode : public rclcpp::Node
{
   public:
    /**
     * @brief Construct a new Gripper Control Node object
     *
     */
    GripperControlNode();
    KeyboardReader keyboard_reader_;

   private:
    double pos;
    bool publish_pos;
    int task_status;
    void timer_callback();
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr task_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // GRIPPER_CONTROL_NODE_HPP
