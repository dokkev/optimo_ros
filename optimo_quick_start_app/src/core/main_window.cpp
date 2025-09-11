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

#include "optimo_quick_start_app/core/main_window.hpp"

#include <QAction>
#include <QLayout>
#include <QMenuBar>

#include <iostream>

namespace optimo_quick_start_app
{
///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// MainWindow ////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

MainWindow::MainWindow()
: main_widget(new QStackedWidget(this)),
  debug_panel(new DebugPanel(this)),
  jog_panel(new JogPanel(this)),
  traj_panel(new TrajPanel(this))
{
  setMaximumSize(1600, 900);
  resize(QSize(1600, 900));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::run()
{
  init();
  show();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::init()
{
  main_widget->insertWidget(static_cast<int>(WidgetIndex::DEBUG_PANEL), debug_panel);
  main_widget->insertWidget(static_cast<int>(WidgetIndex::JOG_PANEL), jog_panel);
  main_widget->insertWidget(static_cast<int>(WidgetIndex::TRAJ_PANEL), traj_panel);

  setCentralWidget(main_widget);
  main_widget->setCurrentIndex(static_cast<int>(WidgetIndex::DEBUG_PANEL));

  init_menubar();
  init_connections();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::init_menubar()
{
  QMenuBar * nav_bar = new QMenuBar(this);
  setMenuBar(nav_bar);

  QAction * go_to_debug_panel = new QAction("Debug Panel", this);
  QAction * go_to_jog_panel = new QAction("Jogging Panel", this);
  QAction * go_to_traj_panel = new QAction("Trajectory Panel", this);

  nav_bar->addAction(go_to_debug_panel);
  nav_bar->addAction(go_to_jog_panel);
  nav_bar->addAction(go_to_traj_panel);

  connect(go_to_debug_panel, &QAction::triggered, this, [this]() {
    main_widget->setCurrentIndex(static_cast<int>(WidgetIndex::DEBUG_PANEL));
  });
  connect(go_to_jog_panel, &QAction::triggered, this, [this]() {
    main_widget->setCurrentIndex(static_cast<int>(WidgetIndex::JOG_PANEL));
  });
  connect(go_to_traj_panel, &QAction::triggered, this, [this]() {
    main_widget->setCurrentIndex(static_cast<int>(WidgetIndex::TRAJ_PANEL));
  });

  // Set the style sheet for the QMenuBar -- temp create a proxy style class later
  QString styleSheet =
    "QMenuBar { background-color: #333; color: white; border: none; }"
    "QMenuBar::item { spacing: 3px; padding: 8px; background: transparent; }"
    "QMenuBar::item:selected { background-color: #555; }"
    "QMenuBar::item:pressed { background-color: #777; }";

  nav_bar->setStyleSheet(styleSheet);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::init_connections()
{
  connect(this, &MainWindow::update_debug_panel, debug_panel, &DebugPanel::update);
  connect(debug_panel, &DebugPanel::enable_btn_pressed, this, &MainWindow::enable_robot);
  connect(debug_panel, &DebugPanel::disable_btn_pressed, this, &MainWindow::disable_robot);
  connect(debug_panel, &DebugPanel::brakes_btn_toggled, this, &MainWindow::toggle_brakes);
  connect(debug_panel, &DebugPanel::start_hardware_btn_pressed, this, &MainWindow::start_hardware);
  connect(debug_panel, &DebugPanel::stop_hardware_btn_pressed, this, &MainWindow::stop_hardware);
  connect(
    debug_panel, &DebugPanel::calibrate_manipulator, this, &MainWindow::calibrate_manipulator);
  connect(debug_panel, &DebugPanel::stop_motion, this, &MainWindow::stop_motion);
  connect(debug_panel, &DebugPanel::setup_calibration, this, &MainWindow::setup_calibration);

  connect(jog_panel, &JogPanel::stop_motion, this, &MainWindow::stop_motion);

  connect(traj_panel, &TrajPanel::disable, this, &MainWindow::disable_robot);
  connect(traj_panel, &TrajPanel::stop_motion, this, &MainWindow::stop_motion);
  connect(traj_panel, &TrajPanel::teach, this, &MainWindow::teach);
  connect(traj_panel, &TrajPanel::play_traj, this, &MainWindow::play_traj);
  connect(traj_panel, &TrajPanel::move_home, this, &MainWindow::move_home);
  connect(traj_panel, &TrajPanel::free_motion, this, &MainWindow::free_motion);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::update_ui(
  const roboligent::OptimoCommunication::OptimoData & data,
  const roboligent::EtherCATCommunication::EtherCATData & ethercat_data)
{
  update_debug_panel(data, ethercat_data);
}

}  // namespace optimo_quick_start_app
