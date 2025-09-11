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

#ifndef OPTIMO_QUICK_START_APP__CORE__MAIN_WINDOW_HPP_
#define OPTIMO_QUICK_START_APP__CORE__MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QStackedWidget>

#include "optimo_quick_start_app/views/debug_panel.hpp"
#include "optimo_quick_start_app/views/jog_panel.hpp"
#include "optimo_quick_start_app/views/traj_panel.hpp"

namespace optimo_quick_start_app
{
/**
 * @brief Gui Application's MainWindow Object
 *
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT
public:
  /**
   * @brief Construct a new Main Window object
   *
   * - Sets Max Size to 1600 x 900
   * - Resizes to 1600 x 900
   *
   */
  MainWindow();
  /**
   * @brief Destroy the Main Window object
   *
   */
  virtual ~MainWindow() = default;
  /**
   * @brief Calls init and then shows the main window
   *
   */
  void run();

signals:
  /**
   * @brief Signal to update Debug Panel to show Optimo's Data
   *
   * @param state
   */
  void update_debug_panel(
    const roboligent::OptimoCommunication::OptimoData & data,
    const roboligent::EtherCATCommunication::EtherCATData & ethercat_data);

  /**
   * @brief Signal to enable Robot
   *
   */
  void enable_robot();

  /**
   * @brief Signal to Disable Robot
   *
   */
  void disable_robot();

  /**
   * @brief Signal to turn on/off the brakes based on the parameter.
   *
   * @param turn_on true to turn on, false to turn off.
   */
  void toggle_brakes(bool turn_on);

  /**
   * @brief Signal to start hardware
   *
   */
  void start_hardware();

  /**
   * @brief Signal to stop hardware
   *
   */
  void stop_hardware();

  /**
   * @brief Signal to stop current callback
   *
   */
  void stop_motion();

  /**
   * @brief Signal to Calibrate Manipulator
   *
   */
  void calibrate_manipulator();

  /**
   * @brief Signal to teach trajectory
   *
   */
  void teach();

  /**
   * @brief Signal to play trajectory
   *
   */
  void play_traj();

  /**
   * @brief Signal to move home
   *
   */
  void move_home();

  /**
   * @brief Signal to start free motion.
   *
   */
  void free_motion();

  /**
   * @brief Signal for starting the setup for calibration
   *
   */
  void setup_calibration();

public slots:
  /**
   * @brief Slot that pings all UI to update their states.
   *
   * @param state
   */
  void update_ui(
    const roboligent::OptimoCommunication::OptimoData & data,
    const roboligent::EtherCATCommunication::EtherCATData & ethercat_data);

private:
  /**
   * @brief Initializes Main Window's Stacked Widget and calls initialises MenuBar and Connections
   * using Helpers
   *
   */
  void init();
  /**
   * @brief Helper that Makes All Connections
   *
   */
  void init_connections();
  /**
   * @brief Initializes and Sets up the Menubar on the top.
   *
   */
  void init_menubar();
  QStackedWidget * main_widget;
  DebugPanel * debug_panel;
  JogPanel * jog_panel;
  TrajPanel * traj_panel;

  /**
   * @brief Stacked Widget Indicies
   *
   */
  enum class WidgetIndex
  {
    DEBUG_PANEL = 0,
    JOG_PANEL,
    TRAJ_PANEL,
    COUNT
  };
};

}  // namespace optimo_quick_start_app

#endif  // OPTIMO_QUICK_START_APP__CORE__MAIN_WINDOW_HPP_
