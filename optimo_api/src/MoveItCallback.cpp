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

#include "optimo_api/MoveItCallback.h"

#include <rl/model/ModelTypes.h>
#include <rl/util/Vector.h>

#include "optimo_api/Shared.h"

namespace optimo_ros
{
MoveItCallback::MoveItCallback(
  roboligent::Model & model_, optimo::TaskQueueObject & task_, MoveItInterface & moveit_iface_)
: roboligent::AbstractCallback(model_),
  timer(roboligent::Timer::Seconds(0)),
  trajectory(model_),
  task(task_),
  state(MoveItState::WAIT),
  moveit_iface(moveit_iface_)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MoveItCallback::calculate_torque(std::vector<int> & torque_)
{
  // set to free motion

  switch (task->get_status()) {
    case roboligent::CommandStatus::IDLE: {  // Use std::async to run the planning operation
                                             // asynchronously
      if (!moveit_iface.is_valid()) {
        LOG_ERROR(
          "MoveItCallback::calculate_torque: Move group interface not active. Please "
          "check if "
          "the move group node was launched.");
        task->set_status(roboligent::CommandStatus::FAILURE);
        return;
      }
      if (!std::holds_alternative<roboligent::Pose>(task->command().command_data)) {
        LOG_ERROR("MoveItCallback::calculate_torque: Command parameters invalid!");
        task->set_status(roboligent::CommandStatus::FAILURE);
        return;
      }
      auto & pose = std::get<roboligent::Pose>(task->command().command_data);
      static auto pose_msg = geometry_msgs::msg::Pose();
      pose_msg.position.x = pose.position.x();  // no init list for ros msgs
      pose_msg.position.y = pose.position.y();
      pose_msg.position.z = pose.position.z();
      pose_msg.orientation.x = pose.orientation.axis().x();  // Our pose =/= ROS pose
      pose_msg.orientation.y = pose.orientation.axis().y();
      pose_msg.orientation.z = pose.orientation.axis().z();
      pose_msg.orientation.w = pose.orientation.angle();
      // Need a way to alter robot command outside of the sdk
      if (task->command().data[0] != 0) goal.param.impedance = task->command().data[0];
      if (task->command().data[1] != 0) goal.param.max_force = task->command().data[1];
      if (task->command().data[2] != 0) goal.param.stop_sensitivity = task->command().data[2];

      result_future = std::async(
        std::launch::async, [&]() { return moveit_iface.send_planning_request(pose_msg); });

      state = MoveItState::WAIT;
      task->set_status(roboligent::CommandStatus::IN_PROGRESS);
      break;
    }
    case roboligent::CommandStatus::IN_PROGRESS: {
      switch (state) {
        case MoveItState::WAIT: {
          torque_ = torque_ + model.get_joint_fallback_impedance_torque();
          if (result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto msg = result_future.get();
            if (!msg.empty()) {
              // FIXME get duration from moveit directly, 20 Hz is MoveIt default
              goal.traj_reader.reset(msg, 20, roboligent::TrajType::JOINT);
              // Reset trajectory
              trajectory.stop();
              if (trajectory.start(goal)) {
                state = MoveItState::FOLLOW;
              } else {
                task->set_status(roboligent::CommandStatus::FAILURE);
              }

              timer.start();
            } else
              task->set_status(roboligent::CommandStatus::FAILURE);
          }
          break;
        }
        case MoveItState::FOLLOW: {
          torque_ = torque_ + trajectory.calculate_torque();
          if (trajectory.get_status() == roboligent::TrajectoryStatus::COMPLETE)
            task->set_status(roboligent::CommandStatus::SUCCESS);
          break;
        }
      }
      break;
    }
  }
}

}  // namespace optimo_ros
