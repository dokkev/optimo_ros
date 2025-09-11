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

#ifndef OPTIMO_QUICK_START_APP__VIEWS__DEBUG_PANEL_HPP_
#define OPTIMO_QUICK_START_APP__VIEWS__DEBUG_PANEL_HPP_

#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

#include <rl/communication/EtherCATCommunication.h>

namespace optimo_quick_start_app
{
/**
 * @brief Debug Panel Class to shows Optimo Robots data. This includes individual joint data,
 * overall communication data, Optimo’s state and also contains buttons to perform calibration,
 * enabling, disabling and engaging and disengaging brakes.
 *
 */
class DebugPanel : public QWidget
{
  Q_OBJECT
public:
  /**
   * @brief Construct a new Debug Panel Object.
   *  - Styling and initialization the widget with required LineEdits, Buttons and Labels.
   *  - Creates Layout and sets up Layout.
   *
   * @param parent parent Widget
   */
  explicit DebugPanel(QWidget * parent);

public slots:
  /**
   * @brief Update Slot to Update Optimo Data using OptimoData and EtherCATData.
   *
   * @param data
   * @param ethercat_data
   */
  void update(
    const roboligent::OptimoCommunication::OptimoData & data,
    const roboligent::EtherCATCommunication::EtherCATData & ethercat_data);

signals:
  /**
   * @brief Signal that triggers when enable_btn is pressed.
   *
   */
  void enable_btn_pressed();

  /**
   * @brief Signal that triggers when disable button is pressed.
   *
   */
  void disable_btn_pressed();

  /**
   * @brief Signal that is emitted when brake button is pressed.
   *
   * @param turn_on true for turn on, false for turn off.
   */
  void brakes_btn_toggled(bool turn_on);

  /**
   * @brief Signal emmitted when start_hardware_btn is pressed.
   *
   */
  void start_hardware_btn_pressed();

  /**
   * @brief Signal emitted when stop_hardware_btn is pressed.
   *
   */
  void stop_hardware_btn_pressed();

  /**
   * @brief Signal to Calibrate the Manipulator
   *
   */
  void calibrate_manipulator();

  /**
   * @brief Signal for stopping current callback
   *
   */
  void stop_motion();

  /**
   * @brief Signal for starting the setup for calibration
   *
   */
  void setup_calibration();

private slots:

  /**
   * @brief Helper that countdowns  3,2,1 using a QProgressDialog before turning off the brakes so
   * that the user is ready
   *
   */
  void brake_off_behaviour();

  /**
   * @brief Calibrate Behaviour is used to disable the calibrate button while the calibration is
   * going on. it also calls the calibrate_manipulator() signal.
   *
   */
  void calibrate_behaviour();

private:
  /**
   * @brief Helper to Style Line Edit
   *
   * @param edit LineEdit to style
   */
  void style_edit(QLineEdit * edit);

  /**
   * @brief QLabel Styling and creating Helper
   *
   * @param text Creates and Styles QLabel
   * @return QLabel* Styled and Initialized QLabel
   */
  QLabel * style_label(QString text);

  QGridLayout * state_grid;
  QLineEdit * connect_edit;
  QLineEdit * configure_edit;
  QLineEdit * status_edit;
  QLineEdit * mode_edit;
  QPushButton * enable_btn;
  QPushButton * disable_btn;
  QPushButton * brakes_on_btn;
  QPushButton * brakes_off_btn;
  QPushButton * start_hardware_btn;
  QPushButton * stop_hardware_btn;
  QPushButton * calibrate_btn;
  QPushButton * setup_calibration_btn;
  QPushButton * stop_btn;

  /**
   * @brief Helper to correctly position Data in the Joint State Grid
   *
   */
  enum class DebugEntry : int
  {
    JOINT = 0,
    STATE,
    OPMODE,
    TARGET,
    TRQ,
    ANGLE,
    VEL,
    ACC,
    SPR_POS,
    MTR_POS,
    COUNT
  };
};
}  // namespace optimo_quick_start_app

#endif  // OPTIMO_QUICK_START_APP__VIEWS__DEBUG_PANEL_HPP_
