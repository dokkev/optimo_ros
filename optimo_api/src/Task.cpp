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

#include "optimo_api/Task.h"

namespace optimo
{
Task::Task(roboligent::RobotCommand & command_, int timeout_)
: task_progress(0.0),
  task_timer(roboligent::Timer::Seconds(timeout_)),
  command_status(roboligent::CommandStatus::QUEUED),
  robot_command(command_)
{
}

// ///////////////////////////////////////////////////////////////////////////////////////////////////

roboligent::RobotCommand & Task::command() { return robot_command; }

///////////////////////////////////////////////////////////////////////////////////////////////////

const roboligent::CommandStatus & Task::get_status() const { return command_status; }

///////////////////////////////////////////////////////////////////////////////////////////////////

void Task::set_status(roboligent::CommandStatus status_)
{
  command_status = status_;
  if (is_completed()) set_progress(1.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool Task::is_completed() const
{
  return (
    command_status == roboligent::CommandStatus::SUCCESS ||
    command_status == roboligent::CommandStatus::FAILURE);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool Task::is_timeout() const { return task_timer.is_timeout(); }

///////////////////////////////////////////////////////////////////////////////////////////////////

void Task::set_progress(double progress) { task_progress = progress; }

///////////////////////////////////////////////////////////////////////////////////////////////////

double Task::get_progress() const { return task_progress; }

///////////////////////////////////////////////////////////////////////////////////////////////////

void Task::start_task()
{
  task_timer.start();
  set_status(roboligent::CommandStatus::IDLE);
}

}  // namespace optimo