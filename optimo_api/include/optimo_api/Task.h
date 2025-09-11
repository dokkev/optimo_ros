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


#ifndef OPTIMO_CONTROLLER_SRC_CONTROLLER_TASK_H_
#define OPTIMO_CONTROLLER_SRC_CONTROLLER_TASK_H_

#include <queue>

#include <chrono>

#include <rl/common/RobotCommand.h>
#include <rl/util/Timer.h>

#define MAKE_SHARED_TASK(x) std::make_shared<optimo::Task>(x)  // x is a roboligent::Task object

namespace optimo
{
/**
 * @brief The Task Class acts as an communication method between the Controller and a higher level
 * commander such as Session. This class is used to tell Controller what to do and how long it has
 * to do it. It is also used to track the progress of the task on the higher level while controller
 * executes them.
 *
 * To make this happen, This call is passed to and throughout the controller and callbacks as a
 * shared pointer (TaskQueueObject). A regular roboligent::Task object can be converted to a
 * TaskQueueObject using the Macro MAKE_SHARED_TASK(my_task_obj).
 *
 * \code
 * roboligent::RobotCommand enable_cmd;
 * enable_cmd.id = static_cast<int>(roboligent::CommandID::ENABLE);
 * TaskQueueObject enable_task = MAKE_SHARED_TASK(roboligent::Task(enable_cmd,
 * roboligent::Timer::NO_TIMEOUT)); controller.command_robot(enable_task); \endcode
 *
 */
class Task
{
public:
  //----------------Initializer------------------//

  /**
   * @brief Construct a new Task object using RobotCommand and Timer Timeout. Use
   * roboligent::Timer::NO_TIMEOUT for no timeout.
   *
   * @param command Command to be executed
   * @param timeout Command Timeout
   */
  explicit Task(roboligent::RobotCommand & command_, int timeout_);

  //----------------Getters & Setters------------------//

  /**
   * @brief Get the RobotCommand Object.
   *
   * @return RobotCommand& command object
   */
  roboligent::RobotCommand & command();

  /**
   * @brief Get the CommandStatus
   *
   * @return const CommandStatus&
   */
  const roboligent::CommandStatus & get_status() const;

  /**
   * @brief Set the status of the Command
   *
   * @param status_ CommandStatus to set to
   */
  void set_status(roboligent::CommandStatus status_);

  /**
   * @brief Set the progress of the command. Should be between 0.0 and 1.0 (0% to 100%)
   *
   * @param progress progress to set.
   */
  void set_progress(double progress);

  /**
   * @brief Get the progress of the command. SHould be between 0.0 to 1.0 (0 to 100%)
   *
   * @return double current progress
   */
  double get_progress() const;

  //----------------Others------------------//

  /**
   * @brief Returns true if the command is completed (CommandStatus is SUCCESS or FAILURE)
   *
   * @return true on command Completion
   * @return false on non-completion of command
   */
  bool is_completed() const;

  /**
   * @brief Returns true if the timer is timeout. Note: Timer never times out if
   * roboligent::Timer::NO_TIMEOUT was used.
   *
   * @return true on timeout
   * @return false if timer has not timed out or no timeout was set.
   */
  bool is_timeout() const;

  /**
   * @brief Starts the Task (Starts Timer & sets the current CommandStatus to IDLE)
   *
   */
  void start_task();

private:
  double task_progress;
  roboligent::Timer task_timer;
  roboligent::CommandStatus command_status;
  roboligent::RobotCommand robot_command;
};

using TaskQueueObject = std::shared_ptr<optimo::Task>;
using TaskQueue = std::queue<optimo::TaskQueueObject>;

}  // namespace optimo
#endif
