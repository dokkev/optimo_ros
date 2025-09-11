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

#include "optimo_quick_start_app/views/debug_panel.hpp"

#include <QCoreApplication>
#include <QProgressDialog>

#include "optimo_quick_start_app/core/session.hpp"
#include "optimo_quick_start_app/style/util.hpp"

namespace optimo_quick_start_app
{
DebugPanel::DebugPanel(QWidget * parent)
: QWidget(parent),
  state_grid(new QGridLayout),
  connect_edit(new QLineEdit(this)),
  configure_edit(new QLineEdit(this)),
  status_edit(new QLineEdit(this)),
  mode_edit(new QLineEdit(this)),
  enable_btn(new QPushButton(this)),
  disable_btn(new QPushButton(this)),
  brakes_on_btn(new QPushButton(this)),
  brakes_off_btn(new QPushButton(this)),
  start_hardware_btn(new QPushButton(this)),
  stop_hardware_btn(new QPushButton(this)),
  calibrate_btn(new QPushButton(this)),
  setup_calibration_btn(new QPushButton(this)),
  stop_btn(new QPushButton(this))
{
  style_edit(connect_edit);
  style_edit(configure_edit);
  style_edit(status_edit);
  style_edit(mode_edit);

  auto top_layout = new QGridLayout;
  top_layout->setSpacing(20);
  top_layout->setAlignment(Qt::AlignTop | Qt::AlignLeft);

  top_layout->addWidget(style_label("Connection"), 0, 0);
  top_layout->addWidget(connect_edit, 0, 1);
  top_layout->setAlignment(connect_edit, Qt::AlignTop | Qt::AlignLeft);

  top_layout->addWidget(style_label("Configuration"), 1, 0);
  top_layout->addWidget(configure_edit, 1, 1);
  top_layout->setAlignment(configure_edit, Qt::AlignTop | Qt::AlignLeft);

  top_layout->addWidget(style_label("Status"), 2, 0);
  top_layout->addWidget(status_edit, 2, 1);
  top_layout->setAlignment(status_edit, Qt::AlignTop | Qt::AlignLeft);

  top_layout->addWidget(style_label("Mode"), 3, 0);
  top_layout->addWidget(mode_edit, 3, 1);
  top_layout->setAlignment(mode_edit, Qt::AlignTop | Qt::AlignLeft);

  std::vector<std::string> DebugStrings{
    "JOINT", "STATE",    "OP MODE",      "TRQ CMD",         "TRQ OUTPUT",
    "ANGLE", "VELOCITY", "ACCELERATION", "SPRING POSITION", "MOTOR POSITION"};

  for (int row = 0; row < static_cast<int>(DebugEntry::COUNT); ++row) {
    for (int col = 0; col <= 7; ++col) {
      if (col == 0) {
        state_grid->addWidget(style_label((DebugStrings[row] + ":").c_str()), row, col);
      } else {
        auto line = new QLineEdit;
        style_edit(line);
        state_grid->addWidget(line, row, col);
      }
    }
  }
  state_grid->setSpacing(20);

  enable_btn->setText("Enable");
  disable_btn->setText("Disable");
  brakes_on_btn->setText("Turn Brakes On");
  brakes_off_btn->setText("Turn Brakes Off");
  calibrate_btn->setText("Calibrate Manipulator");
  setup_calibration_btn->setText("Set Calibration Points");
  stop_btn->setText("Stop Motion");
  start_hardware_btn->setText("Start Hardware");
  stop_hardware_btn->setText("Stop Hardware");
  stop_hardware_btn->setDisabled(true);

  connect(enable_btn, &QPushButton::clicked, this, &DebugPanel::enable_btn_pressed);
  connect(disable_btn, &QPushButton::clicked, this, &DebugPanel::disable_btn_pressed);
  connect(brakes_on_btn, &QPushButton::clicked, [&]() { brakes_btn_toggled(true); });
  connect(brakes_off_btn, &QPushButton::clicked, [&]() { this->brake_off_behaviour(); });
  connect(calibrate_btn, &QPushButton::clicked, [&]() { this->calibrate_behaviour(); });
  connect(setup_calibration_btn, &QPushButton::clicked, this, &DebugPanel::setup_calibration);

  connect(stop_btn, &QPushButton::clicked, this, &DebugPanel::stop_motion);

  connect(start_hardware_btn, &QPushButton::clicked, [&]() {
    start_hardware_btn->setDisabled(true);
    stop_hardware_btn->setDisabled(true);

    start_hardware_btn->setText("Loading...");
    Session::process_events();
    start_hardware_btn_pressed();

    start_hardware_btn->setText("Start Hardware");
    stop_hardware_btn->setEnabled(true);
  });

  connect(stop_hardware_btn, &QPushButton::clicked, [&]() {
    start_hardware_btn->setDisabled(true);
    stop_hardware_btn->setDisabled(true);

    stop_hardware_btn->setText("Loading...");
    Session::process_events();

    stop_hardware_btn_pressed();

    stop_hardware_btn->setText("Stop Hardware");
    start_hardware_btn->setEnabled(true);
  });

  auto btn_layout = new QHBoxLayout;
  btn_layout->addStretch(1);
  btn_layout->addWidget(enable_btn);
  btn_layout->addWidget(disable_btn);
  btn_layout->addWidget(calibrate_btn);
  btn_layout->addWidget(setup_calibration_btn);
  btn_layout->addWidget(stop_btn);
  btn_layout->addWidget(brakes_on_btn);
  btn_layout->addWidget(brakes_off_btn);
  btn_layout->addStretch(1);

  auto layout = new QVBoxLayout;
  layout->addWidget(start_hardware_btn, 0, Qt::AlignHCenter | Qt::AlignTop);
  layout->addWidget(stop_hardware_btn, 0, Qt::AlignHCenter | Qt::AlignTop);

  layout->addLayout(top_layout);
  layout->addSpacing(50);
  layout->addLayout(state_grid);
  layout->addSpacing(50);
  layout->addLayout(btn_layout);
  layout->addStretch(1);

  setLayout(layout);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void DebugPanel::style_edit(QLineEdit * edit)
{
  edit->setReadOnly(true);
  edit->setPlaceholderText("--");
  edit->setFixedSize(200, 25);
  edit->setAlignment(Qt::AlignCenter);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void DebugPanel::calibrate_behaviour()
{
  calibrate_btn->setDisabled(true);
  QCoreApplication::processEvents();
  calibrate_manipulator();
  calibrate_btn->setEnabled(true);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void DebugPanel::brake_off_behaviour()
{
  auto progress = new QProgressDialog("Turning off Brakes", QString(), 0, 3, this);
  progress->setWindowModality(Qt::ApplicationModal);
  progress->setWindowFlag(Qt::WindowStaysOnTopHint);
  progress->setWindowFlag(Qt::FramelessWindowHint);
  progress->show();

  auto one_sec = std::chrono::high_resolution_clock::now() + std::chrono::seconds(1);

  progress->setValue(1);
  progress->setLabelText("Turning off in 3");
  while (std::chrono::high_resolution_clock::now() < one_sec) {
    Session::process_events();
  }

  progress->setValue(2);
  progress->setLabelText("Turning off in 2");
  one_sec = std::chrono::high_resolution_clock::now() + std::chrono::seconds(1);
  while (std::chrono::high_resolution_clock::now() < one_sec) {
    Session::process_events();
  }

  progress->setValue(3);
  progress->setLabelText("Turning off in 1");
  one_sec = std::chrono::high_resolution_clock::now() + std::chrono::seconds(1);
  while (std::chrono::high_resolution_clock::now() < one_sec) {
    Session::process_events();
  }

  brakes_btn_toggled(false);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QLabel * DebugPanel::style_label(QString text) { return new QLabel(text, this); }

///////////////////////////////////////////////////////////////////////////////////////////////////

void DebugPanel::update(
  const roboligent::OptimoCommunication::OptimoData & data,
  const roboligent::EtherCATCommunication::EtherCATData & ethercat_data)
{
  connect_edit->setText(Utils::to_string(data.running).c_str());
  configure_edit->setText(Utils::to_string(data.parameters.configured).c_str());
  status_edit->setText(roboligent::optimo_status_string(data.optimo_status).c_str());
  mode_edit->setText(roboligent::optimo_op_mode_string(data.optimo_op_mode).c_str());

  for (int col = 1; col <= data.arm_dof; ++col) {
    static_cast<QLineEdit *>(
      state_grid->itemAtPosition(static_cast<int>(DebugEntry::JOINT), col)->widget())
      ->setText(("J" + std::to_string(col)).c_str());

    // Fixme make this two diff things?
    auto error =
      (ethercat_data.cia_state[col - 1] != roboligent::CIA402State::FAULT &&
       ethercat_data.cia_state[col - 1] != roboligent::CIA402State::FAULT_REACTION_ACTIVE)
        ? static_cast<int>(ethercat_data.cia_state[col - 1])
        : static_cast<int>(ethercat_data.rx_error[col - 1]);

    static_cast<QLineEdit *>(
      state_grid->itemAtPosition(static_cast<int>(DebugEntry::STATE), col)->widget())
      ->setText(roboligent::joint_error_string(static_cast<roboligent::JointError>(error)).c_str());

    static_cast<QLineEdit *>(
      state_grid->itemAtPosition(static_cast<int>(DebugEntry::OPMODE), col)->widget())
      ->setText(roboligent::joint_opmode_string(
                  static_cast<roboligent::JointOPMode>(ethercat_data.rx_joint_opmode[col - 1]))
                  .c_str());

    static_cast<QLineEdit *>(
      state_grid->itemAtPosition(static_cast<int>(DebugEntry::TARGET), col)->widget())
      ->setText(QString::number(data.tx_data.arm_target_sea_torque[col - 1]));

    static_cast<QLineEdit *>(
      state_grid->itemAtPosition(static_cast<int>(DebugEntry::ANGLE), col)->widget())
      ->setText(QString::number(data.rx_data.arm_position[col - 1]));

    static_cast<QLineEdit *>(
      state_grid->itemAtPosition(static_cast<int>(DebugEntry::VEL), col)->widget())
      ->setText(QString::number(data.rx_data.arm_velocity[col - 1]));

    static_cast<QLineEdit *>(
      state_grid->itemAtPosition(static_cast<int>(DebugEntry::ACC), col)->widget())
      ->setText(QString::number(data.rx_data.arm_acceleration[col - 1]));

    static_cast<QLineEdit *>(
      state_grid->itemAtPosition(static_cast<int>(DebugEntry::TRQ), col)->widget())
      ->setText(QString::number(data.rx_data.arm_sea_torque[col - 1]));

    static_cast<QLineEdit *>(
      state_grid->itemAtPosition(static_cast<int>(DebugEntry::SPR_POS), col)->widget())
      ->setText(QString::number(data.rx_data.arm_spr_position[col - 1]));

    static_cast<QLineEdit *>(
      state_grid->itemAtPosition(static_cast<int>(DebugEntry::MTR_POS), col)->widget())
      ->setText(QString::number(data.rx_data.arm_motor_position[col - 1]));
  }
}

}  // namespace optimo_quick_start_app
