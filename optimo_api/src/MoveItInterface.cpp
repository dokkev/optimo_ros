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

#include "optimo_api/MoveItInterface.h"

#include <random>

#include <rl/util/Logger.h>
#include <rl/util/Vector.h>

#include "optimo_api/Shared.h"

namespace optimo_ros
{
MoveItInterface::MoveItInterface(const roboligent::Model & model_, std::string ns)
: model(model_),
  servo_point(model_.size(), 0),
  servo_point_filt(model_.size(), roboligent::LowPass<double>(5, 0.001)),
  init_state(MoveItInit::IN_PROGRESS),
  node(rclcpp::Node::make_shared("optimo_moveit_interface", ns)),
  servo_timer(roboligent::Timer::Seconds(1))
{
  // this executor running in a thread is necessary to listen to move_group without blocking any
  // other code
  spin_thread = std::thread([this]() {
    init();
    executor.add_node(node);
    executor.spin();
  });
}

///////////////////////////////////////////////////////////////////////////////////////////////////

MoveItInterface::~MoveItInterface()
{
  while (init_state == MoveItInit::IN_PROGRESS) sleep(1);
  executor.cancel();
  spin_thread.join();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool MoveItInterface::is_valid() const { return init_state == MoveItInit::VALID; }

///////////////////////////////////////////////////////////////////////////////////////////////////

void MoveItInterface::add_bounding_box(Eigen::Vector3d size, Eigen::Vector3d pos)
{
  // add bounding obstacles
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_iface->getPlanningFrame();
  static int name_ctr = 0;
  collision_object.id = "box" + std::to_string(name_ctr);
  name_ctr++;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = size[0];
  primitive.dimensions[primitive.BOX_Y] = size[1];
  primitive.dimensions[primitive.BOX_Z] = size[2];
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = pos[0];
  box_pose.position.y = pos[1];
  box_pose.position.z = pos[2];

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface->addCollisionObjects(collision_objects);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<std::vector<double>> MoveItInterface::send_planning_request(
  const geometry_msgs::msg::Pose & pose)
{
  // ensure movegroup node is alive
  if (init_state != MoveItInit::VALID || !move_group_iface)
    return std::vector<std::vector<double>>();

  move_group_iface->setNumPlanningAttempts(500);
  move_group_iface->setMaxVelocityScalingFactor(0.03);
  move_group_iface->setMaxAccelerationScalingFactor(0.01);

  LOG_INFO(
    "MoveItInterface::send_planning_request: Current " +
    pose_to_string(move_group_iface->getCurrentPose().pose));
  LOG_INFO("MoveItInterface::send_planning_request: Target " + pose_to_string(pose));
  move_group_iface->setPoseTarget(pose);
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_iface->plan(msg));
  if (ok) {
    auto joint_msg = msg.trajectory_.joint_trajectory;
    std::vector<std::vector<double>> msg_vec(
      joint_msg.points.size(), std::vector<double>(model.size(), 0));
    for (int i = 0; i < model.size(); ++i) {
      int joint_num = static_cast<int>(joint_msg.joint_names[i].back()) - '0' - 1;
      for (int j = 0; j < joint_msg.points.size(); ++j)
        msg_vec[j][joint_num] = joint_msg.points[j].positions[i];
    }
    return msg_vec;
  } else {
    LOG_ERROR("MoveItInterface::send_planning_request: Failed for " + pose_to_string(pose));
    return std::vector<std::vector<double>>();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool MoveItInterface::get_servo_point(std::vector<double> & servo_point_)
{
  if (init_state != MoveItInit::VALID) return false;
  servo_point_.clear();

  for (int i = 0; i < model.size(); ++i) {
    if (servo_timer.is_timeout()) servo_point_filt[i].reset(servo_point[i]);
    servo_point_.push_back(servo_point_filt[i](servo_point[i]));
  }
  servo_timer.start();
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

std::string MoveItInterface::pose_to_string(const geometry_msgs::msg::Pose & pose)
{
  std::stringstream ss;
  ss << "Position: (" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
     << "), ";
  ss << "Orientation: (" << pose.orientation.x << ", " << pose.orientation.y << ", "
     << pose.orientation.z << ", " << pose.orientation.w << ")";
  return ss.str();
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void MoveItInterface::pub_servo_state()
{
  auto message = sensor_msgs::msg::JointState();
  message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  message.position = servo_timer.is_timeout() ? servo_point = model.get_position() : servo_point;

  if (message.position[1] != 0)
    joint_pub->publish(message);  // weird behavior where model.get_position is 0
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MoveItInterface::init()
{
  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr plan_service =
    node->create_client<moveit_msgs::srv::GetMotionPlan>(
      std::string(node->get_namespace()) + "/plan_kinematic_path");
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_service =
    node->create_client<std_srvs::srv::Trigger>(
      std::string(node->get_namespace()) + "/servo_node/start_servo");

  servo_sub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    std::string(node->get_namespace()) + "/servo_node/joint_trajectory", 10,
    std::bind(&MoveItInterface::topic_callback, this, std::placeholders::_1));
  joint_pub = node->create_publisher<sensor_msgs::msg::JointState>(
    std::string(node->get_namespace()) + "/servo_node/servo_joint_states", 10);
  timer = node->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(&MoveItInterface::pub_servo_state, this));

  if (
    plan_service->wait_for_service(std::chrono::milliseconds(5000)) &&
    servo_service->wait_for_service(std::chrono::milliseconds(5000))) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = servo_service->async_send_request(request);
    if (
      rclcpp::spin_until_future_complete(node, result_future, std::chrono::milliseconds(20000)) ==
        rclcpp::FutureReturnCode::SUCCESS &&
      result_future.get()->success) {
      double time = std::chrono::duration_cast<std::chrono::duration<double>>(
                      std::chrono::system_clock::now() - now)
                      .count();
      LOG_INFO(
        "MoveItInterface:: Movegroup and servo service was found after " + std::to_string(time) +
        " seconds. Ready to plan using MoveIt!");
      move_group_iface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node, moveit::planning_interface::MoveGroupInterface::Options(
                "optimo_arm", "robot_description", std::string(node->get_namespace())));
      planning_scene_interface =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>(
          std::string(node->get_namespace()));
      move_group_iface->startStateMonitor();

      init_state = MoveItInit::VALID;
      return;
    } else
      LOG_WARN("MoveItInterface:: Servo service failed, Moveit planning disabled!");
  } else
    LOG_WARN("MoveItInterface:: Movegroup service was not found, Moveit planning disabled!");
  init_state = MoveItInit::INVALID;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MoveItInterface::topic_callback(const trajectory_msgs::msg::JointTrajectory & msg)
{
  // get position information from the ros_msg

  std::vector<double> servo_point_(model.size(), 0);
  for (int i = 0; i < model.size(); i++) {
    // ROS names the joints from 1-7, and pushes them back in a scrambled order
    int joint_num = static_cast<int>(msg.joint_names[i].back()) - '0' - 1;
    if (std::isnan(msg.points[0].positions[i])) return;
    servo_point_[joint_num] = (msg.points[0].positions[i]);
  }
  // this part works similar to joint fallback's anchoring effect
  auto diff_pos = servo_point_ - model.get_position();
  for (int i = 0; i < model.size(); ++i)
    if (abs(diff_pos[i]) > 0.05)
      servo_point[i] = model.get_position()[i] + 0.05 * roboligent::sign(diff_pos[i]);
    else
      servo_point[i] = servo_point_[i];
}

}  // namespace optimo_ros
