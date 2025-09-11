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

#ifndef OPTIMO_QUICK_START_APP__VIEWS__TRAJ_PANEL_HPP_
#define OPTIMO_QUICK_START_APP__VIEWS__TRAJ_PANEL_HPP_

#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

namespace optimo_quick_start_app
{
/**
 * @brief Panel that allows for trajectory related operations on the robot. Currently has functions
 * to:
 *  - Disable the robot
 *  - Stop the current callback
 *  - Move the robot back to the home position, using a joint trajectory.
 *  - Teach the robot a joint trajectory with a given name.
 *  - Plays a joint trajectory with a given name.
 */
class TrajPanel : public QWidget
{
  Q_OBJECT
public:
  /**
   * @brief Construct a new Trajectory Panel Object.
   *  - Styling and initialization the widget with required LineEdits, Buttons and Labels.
   *  - Creates Layout and sets up Layout.
   *
   * @param parent parent Widget
   */
  explicit TrajPanel(QWidget * parent);

  /**
   * @brief Destroy the Traj Panel object
   *
   */
  virtual ~TrajPanel() = default;

signals:

  /**
   * @brief Signal that triggers when disable button is pressed.
   *
   */
  void disable();

  /**
   * @brief Signal for stopping current callback
   *
   */
  void stop_motion();

  /**
   * @brief Signal for teaching trajectory
   *
   */
  void teach();

  /**
   * @brief Signal for moving to home
   *
   */
  void move_home();

  /**
   * @brief Signal for playing trajectory
   *
   */
  void play_traj();

  /**
   * @brief Signal for starting free motion.
   *
   */
  void free_motion();

private:
  QPushButton * disable_btn;
  QPushButton * stop_btn;
  QPushButton * move_home_btn;
  QPushButton * teach_btn;
  QPushButton * play_traj_btn;
  QPushButton * free_btn;
};
}  // namespace optimo_quick_start_app
#endif  // OPTIMO_QUICK_START_APP__VIEWS__TRAJ_PANEL_HPP_
