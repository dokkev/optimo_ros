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

#ifndef OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_API_INCLUDE_OPTIMO_API_MOVEITINTERFACE_H_
#define OPTIMO_CONTROLLER_ROS_WRAPPER_SRC_OPTIMO_API_INCLUDE_OPTIMO_API_MOVEITINTERFACE_H_

#include <future>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rl/model/Model.h>

#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "optimo_api/Task.h"

namespace optimo_ros
{
/**
 * @brief This is a thin wrapper for Moveit's MoveGroupInterface, used to generate a trajectory and
 * provide joint states to moveit when not in simulation.
 *
 * @todo This class should use the singleton pattern to prevent multiple interfaces from being
 * created in different callbacks.
 */
class MoveItInterface
{
public:
  /**
   * @brief Construct a new MoveIt Interface object
   *
   * @param model_
   */
  MoveItInterface(const roboligent::Model & model_, std::string ns);

  /**
   * @brief Destroy the MoveIt Interface object
   *
   */
  ~MoveItInterface();

  /**
   * @brief Returns true if MoveIt's servo and move group nodes are alive and communicated with.
   * If false, this wrapper overall will do nothing (as opposed to simply crashing, which is the
   * case with Moveit's MoveGroupInterface).
   *
   * @return true if MoveIt was connected to.
   */
  bool is_valid() const;

  /**
   * @brief Adds a collision object to the planning scene with the given size and position. Can be
   * used to send workspace limits.
   *
   * @param size The size of the box in x, y, z direction.
   * @param pos The position of the box in x, y, z direction
   */
  void add_bounding_box(Eigen::Vector3d size, Eigen::Vector3d pos);

  /**
   * @brief Requests for a trajectory with the given pose, and returns the
   * plan in the form of an std::vector. If planning fails, the return vector is empty.
   *
   * @param pose A ROS message containing the target pose. Should be in the world frame.
   *
   * @return std::vector<std::vector<double>> The trajectory plan vector, where each element is a
   * vector with the desired joint positions given in ascending order, in radians. The points are
   * given with at a 10Hz rate, which is what GuidedTrajectory expects.
   */
  std::vector<std::vector<double>> send_planning_request(const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Get the servo point from MoveIt servo, with some additional validity checking and
   * filtering.
   *
   * @details If nothing is published to /servo_node/joint_trajectory, this point will not move
   * from the model's position.
   *
   * @param servo_point_ The servo point, given in joint positions (radians). The output of this
   * function.
   *
   * @return true if a valid point should be expected from servo_point_. Can return false if
   * MoveIt was never connected (is_valid() returns false).
   */
  bool get_servo_point(std::vector<double> & servo_point_);

private:
  /**
   * @brief Converts a geometry_msgs::msg::Pose to a string.
   *
   * @param pose
   *
   * @return std::string
   */
  static std::string pose_to_string(const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Initializes the robot, separated in order to perform this action asynchronously.
   * Does not return anything, validity status is held in the valid flag (is_valid()).
   */
  void init();

  /**
   * @brief Used to listen to the servo node, for the latest servo point.
   *
   * @param msg the servo point, in joint trajectory form.
   */
  void topic_callback(const trajectory_msgs::msg::JointTrajectory & msg);

  /**
   * @brief Publish a point for the servo node to base its command off of.
   *
   * @details See this: https://github.com/moveit/moveit2/pull/2349
   *
   */
  void pub_servo_state();

  /**
   * @brief Holds the status of the MoveIt connection
   *
   */
  enum class MoveItInit : int
  {
    IN_PROGRESS,
    VALID,
    INVALID
  } init_state;

  const roboligent::Model & model;
  std::vector<double> servo_point;
  std::vector<roboligent::LowPass<double>> servo_point_filt;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_sub;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_iface;
  roboligent::Timer servo_timer;
  std::thread spin_thread;
};
}  // namespace optimo_ros

#endif
