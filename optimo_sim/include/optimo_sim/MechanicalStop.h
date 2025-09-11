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

#ifndef OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_SIM_INCLUDE_OPTIMO_SIM_MECHANICALSTOP_H_
#define OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_SIM_INCLUDE_OPTIMO_SIM_MECHANICALSTOP_H_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/System.hh>

namespace optimo_ros
{
/**
 * @brief This Gazebo plugin makes the optimo arm's position fixed when torque is set to zero. As we
 * inherit from Gazebo::System, see
 * https://github.com/gazebosim/ros_gz_project_template/blob/main/ros_gz_example_gazebo/include/ros_gz_example_gazebo/FullSystem.hh
 */
class MechanicalStop : public ignition::gazebo::System,
                       public ignition::gazebo::ISystemConfigure,
                       public ignition::gazebo::ISystemPreUpdate,
                       public ignition::gazebo::ISystemUpdate
{
public:
  MechanicalStop() = default;
  /**
   * @brief Initializes joint positions to initial state defined in URDF.
   * @details Plugins inheriting ISystemConfigure must implement the Configure
   * callback. This is called when a system is initially loaded.
   * @param _entity variable contains the entity that the system is attached to
   * @param _element variable contains the sdf Element with custom configuration
   * @param _ecm provides an interface to all entities and components
   * @param _eventManager provides a mechanism for registering internal signals
   */
  void Configure(
    const ignition::gazebo::Entity & _entity, const std::shared_ptr<const sdf::Element> & _element,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & _eventManager) override;

  /**
   * @brief Detects if no joint torque is given. Plugins inheriting ISystemPreUpdate must
   * implement the PreUpdate callback. This is called at every simulation iteration before the
   * physics updates the world.
   *
   * @param _info variable provides information such as time.
   * @param _ecm provides an interface to all entities and components in simulation.
   */
  void PreUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    ignition::gazebo::EntityComponentManager & _ecm) override;

  /**
   * @brief Fixes joint position if no joint torque is given. Plugins inheriting ISystemUpdate
   * must implement the Update callback. This is called at every simulation iteration before
   * the physics updates the world.
   *
   * @param _info variable provides information such as time.
   * @param _ecm provides an interface to all entities and components in simulation.
   */
  void Update(
    const ignition::gazebo::UpdateInfo & _info,
    ignition::gazebo::EntityComponentManager & _ecm) override;

private:
  std::vector<ignition::gazebo::Joint> jointEntity;
  std::vector<double> prev_pos;
  bool no_cmd;
};

}  // namespace optimo_ros

#endif
