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

#ifndef OPTIMO_QUICK_START_APP__CORE__SESSION_HPP_
#define OPTIMO_QUICK_START_APP__CORE__SESSION_HPP_

#include <string.h>

#include <QObject>
#include <QTimer>

#include <memory>

#include "optimo_quick_start_app/core/main_window.hpp"
#include "optimo_quick_start_app/core/optimo_quick_start_node.hpp"

namespace optimo_quick_start_app
{
/**
 * @brief Session for GUI contains the main control loop and is also responsible for spinning the
 * ROS2 node.
 *
 */
class Session : public QObject
{
  Q_OBJECT
public:
  /**
   * @brief Construct a new Session object. Initializes the loop timer and connections.
   *
   * @param node
   * @param main_window
   */
  explicit Session(std::shared_ptr<OptimoQuickStartNode> node, MainWindow & main_window);

  /** @brief Default Destructor. */
  virtual ~Session() = default;

  /**
   * @brief Runs QCoreApplication::processEvents() if time between last call and current call is
   * more than 1 millisecond.
   *
   */
  static void process_events();

public slots:
  /**
   * @brief Enables or disables the robot.
   *
   * @param enable true to enable, false to disable.
   */
  void enable_robot(bool enable);

  /**
   * @brief Toggles brakes.
   *
   * @param turn_on true to turn on, false to turn off.
   */
  void toggle_brakes(bool turn_on);

  /**
   * @brief Starts Sessions
   *
   * - Starts Session Timer
   *
   */
  void start_session();

  /**
   * @brief Stops Session
   *
   *  - Stops Session Timer
   *
   */
  void stop_session();

  /**
   * @brief Calls Calibration Command, Waits for it to complete, flashes the file to J1.
   *
   */
  void calibrate_manipulator();

  /**
   * @brief Creates a text dialog to define the name for a recorded trajectory, then commands to
   * record said trajectory.
   *
   */
  void teach();

  /**
   * @brief Commands controller to start free motion.
   *
   */
  void free_motion();

  /**
   * @brief Creates a text dialog to define the name for the trajectory to be played, then
   * commands to play said trajectory.
   *
   */
  void play_traj();

  /**
   * @brief Commands robot to move to home position.
   *
   */
  void move_home();

  /**
   * @brief Interrupts the current task and returns the robot to idle callback.
   *
   */
  void stop_motion();

  /**
   * @brief Starts Record Calibration Callback and hooks up GUI elements to it to add data points
   * for recording.
   *
   */
  void setup_calibration();

signals:
  /**
   * @brief Update UI signal to Tell MainWindow to Update UI
   *
   * @param state
   */
  void update_ui(
    const roboligent::OptimoCommunication::OptimoData & data,
    const roboligent::EtherCATCommunication::EtherCATData & ethercat_data);

private:
  /**
   * @brief Helper to initialize Connections
   *
   */
  void init_connections();

  /**
   * @brief Update Loop of Session
   *
   */
  void update();

  QTimer * update_timer_;
  MainWindow & main_window_;
  std::shared_ptr<OptimoQuickStartNode> node_;
};

}  // namespace optimo_quick_start_app

#endif  // OPTIMO_QUICK_START_APP__CORE__SESSION_HPP_
