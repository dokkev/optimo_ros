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

#include "optimo_controllers/GripperKeyboard.h"

KeyboardReader::KeyboardReader() : kfd(0)
{
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  struct termios raw;
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
}

void KeyboardReader::readOne(char * c)
{
  int rc = read(kfd, c, 1);
  if (rc < 0) {
    throw std::runtime_error("read failed");
  }
}

void KeyboardReader::shutdown() { tcsetattr(kfd, TCSANOW, &cooked); }

GripperControlNode::GripperControlNode() : Node("gripper_control_node")
{
  publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    std::string(get_namespace()) + "/optimo_gripper_position_controller/commands", 10);
  task_publisher_ =
    this->create_publisher<std_msgs::msg::Int32>(std::string(get_namespace()) + "/task_status", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&GripperControlNode::timer_callback, this));
  pos = 0;
  task_status = 1;
  publish_pos = false;
  RCLCPP_INFO(this->get_logger(), "Gripper Control Node has been started.");
}

void GripperControlNode::timer_callback()
{
  auto message = std_msgs::msg::Float64MultiArray();
  message.data.resize(1);

  try {
    char key;
    keyboard_reader_.readOne(&key);
    if (key == 67) {
      publish_pos = true;
      std::cout << "Now publishing gripper commands" << std::endl;
    } else if (key == 68) {
      publish_pos = false;
      std::cout << "Stopping gripper commands" << std::endl;
    } else if (key == 65)  // Up arrow key
    {
      pos += 0.004;
    } else if (key == 66)  // Down arrow key
    {
      pos -= 0.004;
    } else if (key == 49)  // 1
    {
      task_status = 1;
      std::cout << task_status << std::endl;
    } else if (key == 50)  // 2
    {
      task_status = 2;
      std::cout << task_status << std::endl;
    } else if (key == 51)  // 3
    {
      task_status = 3;
      std::cout << task_status << std::endl;
    }
  } catch (const std::runtime_error & e) {
  }
  if (pos < 0) pos = 0;
  if (pos > 0.12) pos = 0.12;
  message.data[0] = pos;
  if (publish_pos) publisher_->publish(message);
  auto message_task = std_msgs::msg::Int32();
  message_task.data = task_status;
  task_publisher_->publish(message_task);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::cout << "Usage:" << std::endl;
  std::cout << "Left Arrow: Stop cmd publishing" << std::endl;
  std::cout << "Right Arrow: Start cmd publishing" << std::endl;
  std::cout << "Up Arrow: Increment position cmd" << std::endl;
  std::cout << "Down Arrow: Decrement position cmd" << std::endl;
  std::cout << "Number Keys: Set Status" << std::endl;

  auto node = std::make_shared<GripperControlNode>();
  rclcpp::spin(node);
  node->keyboard_reader_.shutdown();
  rclcpp::shutdown();
  return 0;
}
