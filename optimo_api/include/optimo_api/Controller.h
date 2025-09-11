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

#ifndef OPTIMO_CONTROLLER_SRC_CONTROLLER_CONTROLLER_H_
#define OPTIMO_CONTROLLER_SRC_CONTROLLER_CONTROLLER_H_

#include <queue>

#include <rl/communication/EtherCATCommunication.h>
#include <rl/controller/AbstractController.h>

#include "optimo_api/CallbackCommander.h"
#include "optimo_api/IdleCallback.h"
#include "optimo_api/Task.h"

namespace optimo
{
/**
 * @brief Example/Derived Controller Class. To see how to use, see Controller Example.
 */
class Controller : public roboligent::AbstractController
{
public:
  /**
   * @brief Construct a new Controller object using a shared pointer.
   *
   * @param communication EtherCATCommunication
   */
  explicit Controller(std::shared_ptr<roboligent::EtherCATCommunication> & communication_);

  /**
   * @brief Destroy the Controller object. Joins the thread.
   *
   */
  virtual ~Controller() = default;

  /**
   * @brief Get the current robot state from the interface
   *
   * @return const roboligent::RobotState&
   */
  const roboligent::OptimoCommunication::OptimoData & get_data() const;

  /**
   * @brief Get the ethercat data object
   *
   * @return const roboligent::EtherCATCommunication::EtherCATData&
   */
  const roboligent::EtherCATCommunication::EtherCATData & get_ethercat_data() const;

  /**
   * @brief Get the model object
   *
   * @return const roboligent::Model&
   */
  const roboligent::Model & get_model() const;

  /**
   * @brief pushes the task to the task queue
   *
   * @param task_
   */
  void command_robot(optimo::TaskQueueObject task_);

  /**
   * @brief Start Controller starts the communication and throws a runtime error on failure to
   * start communication.
   *
   * @logs
   *
   *  - LOG_ERROR on failure to start
   *
   *
   */
  virtual void start_controller() override;

  /**
   * @brief Stops the update loop, model and communication.
   *
   */
  virtual void end_controller() override;

  /**
   * @brief Uses Communication object to turn brakes on/off based on the parameter (true=on,
   * false=off).
   *
   *
   * @param turn_on true to turn on, false to turn off
   */
  virtual void toggle_brakes(bool turn_on);

  /**
   * @brief Once Calibration is done, this method can be called. It renames the current yaml file
   * to include time and date of change. It then renames the new (calibrated) yaml file to the
   * standard configuration is expecting. It then flashes the new calibration to J1 using
   * HardwareManager.
   *
   * @return true on success
   * @return false on failure
   */
  virtual bool flash_yaml();

  /**
   * @brief Get the current task object
   *
   * @return TaskQueueObject
   */
  TaskQueueObject get_current_task();

  /**
   * @brief Interrupts the current task with the given command, and clears the task queue. Only
   * accepts STOP_MOTION and DISABLE as preemptible commands.
   *@param cmd Should contain STOP_MOTION or DISABLE as the command id.
   *
   */
  void preempt_task(roboligent::RobotCommand cmd);

protected:
  /**
   * @brief Update Function called at 1 kHz.
   *
   *  - Reads the data from hardware.
   *  - Updates current task
   *  - Updates current state
   *  - Uses the callback commander to get the torque command.
   *  - Sets the torque command to the communication
   *  - Writes to communication to transmit the data to hardware.
   *
   * @internal Do not add anything from roboligent::HardwareManager in this Controller::update
   * until OptimoCommunication is Implemented.
   */
  virtual void update() override;

  /**
   * @brief Starts the current task.
   *
   * @logs
   *  - LOG_INFO() on starting new tasks.
   *  - LOG_ERROR() on invalid command id.
   *
   */
  virtual void update_task();

  optimo::TaskQueueObject current_task;
  roboligent::Model model;
  std::shared_ptr<optimo::CallbackCommander> callback_commander;
  std::vector<int> torque_command;

private:
  /**
   * @brief Checks the following
   *
   *  - If there is a ongoing task, that is not completed or timed out, it returns false.
   *  - If there is an ongoing task that is timed out, it sets the task status to
   *    roboligent::CommandStatus::FAILURE.
   *  - If the task has completed, it calls complete_task().
   *  - If there is no current_task or the current task has ended (either due to task ending or
   *    timing out), we check if there are more tasks on the task queue. If there are no more
   *    tasks, it returns false. Otherwise, we take the task from the front of the queue and set
   *    it to current_task, call init_task() and then return true.
   *
   * @return true if new task is set and available
   * @return false if there is no new task or prev task has not finished yet
   */
  bool is_task_configured();

  /**
   * @brief Updates Robot State.
   *
   */
  void update_state();

  /**
   * @brief Pops the current task from the queue resets current_task object so that it can be
   * reassigned when a new task is available. Changes the active callback to IDLE. Sets SG
   * Compensation.
   *
   */
  void complete_task();

  /**
   * @brief Helper for update task that can be used to initialize the command before the callback
   * is set.
   *
   */
  void init_task();

  optimo::TaskQueue task_queue;
  std::shared_ptr<roboligent::EtherCATCommunication> communication;
};
}  // namespace optimo
#endif
