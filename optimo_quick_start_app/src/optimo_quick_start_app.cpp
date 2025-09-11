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

#include <QApplication>

#include "optimo_quick_start_app/core/main_window.hpp"
#include "optimo_quick_start_app/core/optimo_quick_start_node.hpp"
#include "optimo_quick_start_app/core/session.hpp"

void handle_signal(int) { QCoreApplication::exit(); }

int main(int argc, char ** argv)
{
  // Initialize the ROS2 system
  rclcpp::init(argc, argv);

  // Create your ROS2 node
  auto node = std::make_shared<optimo_quick_start_app::OptimoQuickStartNode>();

  // Create Qt application
  QApplication app(argc, argv);

  // signal handler
  std::signal(SIGINT, handle_signal);

  // Create main window
  optimo_quick_start_app::MainWindow window;

  // Create session
  optimo_quick_start_app::Session session(node, window);

  // Show main window
  window.run();

  // Start Qt event loop
  int ret = app.exec();

  // Stop session
  session.stop_session();

  // Shutdown ROS2
  rclcpp::shutdown();

  return ret;
}
