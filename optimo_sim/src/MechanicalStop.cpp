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

#include "optimo_sim/MechanicalStop.h"

#include <string>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/plugin/Register.hh>

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
  optimo_ros::MechanicalStop, ignition::gazebo::System,
  optimo_ros::MechanicalStop::ISystemConfigure, optimo_ros::MechanicalStop::ISystemPreUpdate,
  optimo_ros::MechanicalStop::ISystemUpdate)

namespace optimo_ros
{

void MechanicalStop::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager &)
{
  // Create model object to access convenient functions
  auto model = ignition::gazebo::Model(_entity);

  // default should be similar/equal to rest position in OR
  std::vector<double> init_pos;
  std::string init_pos_str = _sdf->Get<std::string>("init_pos");
  std::string prefix_str = _sdf->Get<std::string>("prefix");

  std::istringstream iss(init_pos_str);
  for (int i = 0; i < 7; i++) {
    double num;
    if (!(iss >> num)) {
      // Handle parsing error, e.g., by setting a default value
      ignerr << "optimo_ros::MechanicalStop::Configure: Initial position not found, moving to "
                "default."
             << std::endl;
      init_pos = {0, 3.3, 0, -2.36, 0, -1.13, 0};
      break;
    } else {
      init_pos.push_back(num);
    }
  }

  prev_pos = init_pos;
  no_cmd = true;

  // Init joint
  for (int i = 0; i < 7; i++) {
    jointEntity.push_back(ignition::gazebo::Joint(
      model.JointByName(_ecm, prefix_str + "joint" + std::to_string(i + 1))));
    jointEntity[i].ResetPosition(_ecm, std::vector<double>{init_pos[i]});
  }

  ignmsg << "optimo_ros::MechanicalStop::Configure: Entity name is " << _entity << std::endl;
}

void MechanicalStop::PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & _ecm)
{
  // if there are no torque commands, lock the joints by saving the last joint position and
  // resetting it to there continuously.
  bool set_cmd = true;
  for (int i = 0; i < jointEntity.size(); i++) {
    set_cmd = _ecm.Component<ignition::gazebo::components::JointForceCmd>(jointEntity[i].Entity())
                    ->Data()[0] == 0
                ? set_cmd
                : false;
  }
  no_cmd = set_cmd;
}

void MechanicalStop::Update(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & _ecm)
{
  if (no_cmd) {
    for (long unsigned int i = 0; i < jointEntity.size(); i++) {
      jointEntity[i].ResetPosition(_ecm, std::vector<double>{prev_pos[i]});
      jointEntity[i].ResetVelocity(_ecm, std::vector<double>{0});
    }
  } else {
    for (long unsigned int i = 0; i < jointEntity.size(); i++) {
      prev_pos[i] = jointEntity[i].Position(_ecm).value().at(0);
    }
  }
}

}  // namespace optimo_ros
