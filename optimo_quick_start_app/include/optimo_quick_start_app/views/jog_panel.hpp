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

#ifndef OPTIMO_QUICK_START_APP__VIEWS__JOG_PANEL_HPP_
#define OPTIMO_QUICK_START_APP__VIEWS__JOG_PANEL_HPP_

#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

namespace optimo_quick_start_app
{
/**
 * @brief Panel that allows for jogging of the robot. Currently only uses the gripper.
 *
 */
class JogPanel : public QWidget
{
  Q_OBJECT
public:
  /**
   * @brief Construct a new Jog Panel Object.
   *  - Styling and initialization the widget with required LineEdits, Buttons and Labels.
   *  - Creates Layout and sets up Layout.
   *
   * @param parent parent Widget
   */
  explicit JogPanel(QWidget * parent);

signals:

  /**
   * @brief Signal for stopping current callback
   *
   */
  void stop_motion();

private:
  QPushButton * stop_btn;
  QPushButton * jog_btn;
  QPushButton * grasp_btn;
  QPushButton * relax_btn;
  QPushButton * release_btn;
};
}  // namespace optimo_quick_start_app
#endif  // OPTIMO_QUICK_START_APP__VIEWS__JOG_PANEL_HPP_
