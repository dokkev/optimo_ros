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

#include "optimo_quick_start_app/core/session.hpp"

#include <QCoreApplication>
#include <QInputDialog>

#include <rl/util/Timer.h>

namespace optimo_quick_start_app
{
///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Session ///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

Session::Session(std::shared_ptr<OptimoQuickStartNode> node, MainWindow & main_window)
: QObject(), update_timer_(new QTimer(this)), main_window_(main_window), node_(node)
{  // Connect the timer's timeout signal to our custom slot
  connect(update_timer_, &QTimer::timeout, this, &Session::update);
  // Set the timer interval to achieve a 50Hz update rate (20 milliseconds)
  update_timer_->setInterval(20);
  init_connections();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::start_session() { update_timer_->start(); }

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::stop_session()
{
  update_timer_->stop();
  enable_robot(false);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::update()
{
  // update_ui(data, ethercat_data);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::init_connections()
{
  connect(this, &Session::update_ui, &main_window_, &MainWindow::update_ui);
  connect(&main_window_, &MainWindow::enable_robot, this, [this]() { enable_robot(true); });
  connect(&main_window_, &MainWindow::disable_robot, this, [this]() { enable_robot(false); });
  connect(&main_window_, &MainWindow::toggle_brakes, this, &Session::toggle_brakes);
  connect(&main_window_, &MainWindow::start_hardware, this, &Session::start_session);
  connect(&main_window_, &MainWindow::stop_hardware, this, &Session::stop_session);
  connect(&main_window_, &MainWindow::calibrate_manipulator, this, &Session::calibrate_manipulator);
  connect(&main_window_, &MainWindow::stop_motion, this, &Session::stop_motion);
  connect(&main_window_, &MainWindow::play_traj, this, &Session::play_traj);
  connect(&main_window_, &MainWindow::teach, this, &Session::teach);
  connect(&main_window_, &MainWindow::move_home, this, &Session::move_home);
  connect(&main_window_, &MainWindow::free_motion, this, &Session::free_motion);
  connect(&main_window_, &MainWindow::setup_calibration, this, &Session::setup_calibration);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::process_events()
{
  static roboligent::Timer timer = roboligent::Timer::Milliseconds(1);
  timer.get_elapsed();
  if (timer.is_timeout()) {
    QCoreApplication::processEvents();
    timer.start();
  } else {
    usleep(1);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::enable_robot(bool enable) { Q_UNUSED(enable); }

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::toggle_brakes(bool turn_on) { Q_UNUSED(turn_on); }

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::setup_calibration() {}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::calibrate_manipulator() {}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::stop_motion() {}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::free_motion() {}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::teach()
{
  QString filename = QInputDialog::getText(nullptr, "Enter File Name", "File Name:");

  if (filename.isEmpty()) return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::play_traj()
{
  QString filename = QInputDialog::getText(nullptr, "Enter File Name", "File Name:");

  if (filename.isEmpty()) return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Session::move_home() {}

}  // namespace optimo_quick_start_app
