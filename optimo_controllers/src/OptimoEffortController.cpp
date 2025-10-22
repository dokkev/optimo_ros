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

#include "optimo_controllers/OptimoEffortController.h"

#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <string>

#include <rl/common/RobotConfiguration.h>
#include <rl/util/Logger.h>
#include <rl/util/Vector.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "optimo_api/FreeMotionCallback.h"
#include "optimo_api/IdleCallback.h"
#include "optimo_api/MoveHomeCallback.h"
#include "optimo_api/MoveItCallback.h"
#include "optimo_api/MoveItServoCallback.h"
#include "optimo_api/PlayTrajCallback.h"
#include "optimo_api/ROSShared.h"
#include "optimo_api/Resource.h"
#include "optimo_api/ServoCallback.h"
#include "optimo_api/ServoFbCallback.h"
#include "optimo_api/TeachCallback.h"

namespace optimo_ros
{
controller_interface::InterfaceConfiguration
OptimoEffortController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= model->size(); ++i) {
    config.names.push_back(robot_prefix + "joint" + std::to_string(i) + "/effort");
  }
  config.names.push_back("/model_safety_error");

  return config;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

controller_interface::InterfaceConfiguration OptimoEffortController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= model->size(); ++i) {
    config.names.push_back(robot_prefix + "joint" + std::to_string(i) + "/position");
    config.names.push_back(robot_prefix + "joint" + std::to_string(i) + "/velocity");
    config.names.push_back(robot_prefix + "joint" + std::to_string(i) + "/effort");
  }

  config.names.push_back("/communication_enabled");

  return config;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

controller_interface::return_type OptimoEffortController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // divide state interface back into pos/vel/effort
  for (auto & state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == "communication_enabled") {
      communication_enabled = (bool)state_interface.get_value();
      continue;
    }
    int joint_num = state_interface.get_prefix_name().back() - '0';
    if (joint_num > 0 || joint_num <= model->size()) {
      if (state_interface.get_interface_name() == "position")
        pos[joint_num - 1] = state_interface.get_value() * roboligent::RAD2DEG;
      if (state_interface.get_interface_name() == "velocity")
        vel[joint_num - 1] = state_interface.get_value() * roboligent::RAD2DEG;
      if (state_interface.get_interface_name() == "effort")
        eff[joint_num - 1] = state_interface.get_value() * roboligent::UNIT2MILLI;
    }
  }

  model->update(pos, vel, std::vector<double>(model->size(), 0), torque_command);

  // This is a hack to allow the controller to disable hw, in the case of poor robot behavior.
  command_interfaces_.back().set_value(model->get_safety_error());

  // adhoc method of resetting fallback on the first run. This behavior should move to model
  // itself
  if (first_run) {
    first_run = false;
    model->reset_fallback();
    model->enable_fallback_defaults(true);
    model->enable_fallback_dragging(true);
    model->enable_elbow_fallback_dragging(true);

    model->enable_j1j7_sg_compensation(false);
    model->enable_j2_sg_compensation(false);
    model->enable_j4_sg_compensation(false);
    model->enable_j6_sg_compensation(false);
    model->set_angle_j6_sg_compensation(0);
  }

  update_task();

  // run the callback defined by current task
  task_mtx.lock();
  int cmd_id =
    current_task ? current_task->command().id : static_cast<int>(ROSExtendedCommandID::NONE);
  task_mtx.unlock();
  try {
    cb_list.at(cmd_id)->get_torque(torque_command);
  } catch (const std::out_of_range & e) {
    cb_list.at(static_cast<int>(ROSExtendedCommandID::NONE))->get_torque(torque_command);
  }
  for (int i = 0; i < model->size(); ++i)
    command_interfaces_[i].set_value(torque_command[i] * roboligent::MILLI2UNIT);

  // If communication transitions from enabled to disabled, use stop_cb to stop all current
  // callbacks.
  static bool prev_communication_enabled = true;
  if (prev_communication_enabled && !communication_enabled) {
    stop_cb(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      std::make_shared<std_srvs::srv::Trigger::Response>());
  }
  prev_communication_enabled = communication_enabled;

  //////////////////////////////////////////////////////////////////////////
  // Publish end-effector pose
  optimo_msgs::msg::PoseElbow ee_pose_msg;

  // Get TCP pose from model
  eef_point = model->get_tcp_pose();

  // Set translation
  ee_pose_msg.pose.position.x = eef_point[0];
  ee_pose_msg.pose.position.y = eef_point[1];
  ee_pose_msg.pose.position.z = eef_point[2];

  // Convert custom orientation representation to quaternion

  // Extract target Z-axis from eef_point
  Eigen::Vector3d target_z_axis(eef_point[3], eef_point[4], eef_point[5]);

  // Get J6's current frame X-axis
  // Get J6's current frame X-axis
  Eigen::Vector3d J6_x_axis = model->get_rotation_eigen(6).col(0);

  // Project J6's X-axis onto the plane perpendicular to target_z_axis
  Eigen::Vector3d proj = (J6_x_axis - target_z_axis * target_z_axis.dot(J6_x_axis)).normalized();

  // Reconstruct the X-axis using the stored angle
  double angle = eef_point[6];
  Eigen::Vector3d target_x_axis = proj * std::cos(angle) + target_z_axis.cross(proj) *
  std::sin(angle);

  // Compute the Y-axis as the cross product of Z and X to ensure a right-handed frame
  Eigen::Vector3d target_y_axis = target_z_axis.cross(target_x_axis).normalized();

  // Construct the rotation matrix
  Eigen::Matrix3d R;
  R.col(0) = target_x_axis;
  R.col(1) = target_y_axis;
  R.col(2) = target_z_axis;

  // Convert rotation matrix to quaternion
  Eigen::Quaterniond q(R);

  // Assign quaternion values to the pose message
  ee_pose_msg.pose.orientation.w = q.w();
  ee_pose_msg.pose.orientation.x = q.x();
  ee_pose_msg.pose.orientation.y = q.y();
  ee_pose_msg.pose.orientation.z = q.z();

  // Assign elbow angle
  ee_pose_msg.elbow_angle = model->get_elbow_pose().angle;

  // Publish end-effector pose message
  ee_pose_pub_->publish(ee_pose_msg);

  // // Create and publish the TF
  // geometry_msgs::msg::TransformStamped transform_stamped;
  // transform_stamped.header.stamp = get_node()->get_clock()->now();
  // transform_stamped.header.frame_id = "stand";
  // transform_stamped.child_frame_id = "end_effector";

  // // Set translation
  // transform_stamped.transform.translation.x = eef_point[0];
  // transform_stamped.transform.translation.y = eef_point[1];
  // transform_stamped.transform.translation.z = eef_point[2];

  // // Copy orientation (already converted to quaternion)
  // transform_stamped.transform.rotation.w = ee_pose_msg.pose.orientation.w;
  // transform_stamped.transform.rotation.x = ee_pose_msg.pose.orientation.x;
  // transform_stamped.transform.rotation.y = ee_pose_msg.pose.orientation.y;
  // transform_stamped.transform.rotation.z = ee_pose_msg.pose.orientation.z;

  // // Publish the transform
  // tf_broadcaster_->sendTransform(transform_stamped);

  // Eigen::Matrix<double, 7, 6> J = model->get_jacobian();

  // // publish jacobian
  // std_msgs::msg::Float64MultiArray jacobian_msg;

  // // Specify layout (rows × cols)
  // jacobian_msg.layout.dim.resize(2);
  // jacobian_msg.layout.dim[0].label = "rows";
  // jacobian_msg.layout.dim[0].size = J.rows();
  // jacobian_msg.layout.dim[0].stride = J.rows() * J.cols();
  // jacobian_msg.layout.dim[1].label = "cols";
  // jacobian_msg.layout.dim[1].size = J.cols();
  // jacobian_msg.layout.dim[1].stride = J.cols();

  // // Fill row-major data
  // jacobian_msg.data.resize(J.size());
  // for (int r = 0; r < J.rows(); ++r)
  //   for (int c = 0; c < J.cols(); ++c)
  //     jacobian_msg.data[r * J.cols() + c] = J(r, c);

  // jacobian_pub_->publish(jacobian_msg);

  ///////////////////////////////////////////////////

  // ------------------------------------------------------------------

  Eigen::Vector3d ext_force = model->get_ee_force();
  geometry_msgs::msg::WrenchStamped ext_force_msg;
  ext_force_msg.header.stamp = get_node()->get_clock()->now();
  ext_force_msg.header.frame_id = "end_effector";
  ext_force_msg.wrench.force.x = ext_force[0];
  ext_force_msg.wrench.force.y = ext_force[1];
  ext_force_msg.wrench.force.z = ext_force[2];
  ext_force_msg.wrench.torque.x = 0.0;  // Assuming no torque for external force
  ext_force_msg.wrench.torque.y = 0.0;
  ext_force_msg.wrench.torque.z = 0.0;  // Assuming no torque for external force

  ext_wrench_pub_->publish(ext_force_msg);

  // Use the listener to get world→ee
  try {
    const auto now = get_node()->get_clock()->now();

    // Get world→ee (composes static + dynamic as needed)
    geometry_msgs::msg::TransformStamped T_world_ee =
      tf_buffer_->lookupTransform("stand", "ee", rclcpp::Time(0));

    // Publish PoseElbow
    optimo_msgs::msg::PoseElbow ee_pose_msg;
    ee_pose_msg.pose.position.x = T_world_ee.transform.translation.x;
    ee_pose_msg.pose.position.y = T_world_ee.transform.translation.y;
    ee_pose_msg.pose.position.z = T_world_ee.transform.translation.z;
    ee_pose_msg.pose.orientation.w = T_world_ee.transform.rotation.w;
    ee_pose_msg.pose.orientation.x = T_world_ee.transform.rotation.x;
    ee_pose_msg.pose.orientation.y = T_world_ee.transform.rotation.y;
    ee_pose_msg.pose.orientation.z = T_world_ee.transform.rotation.z;
    // keep your elbow field if applicable
    ee_pose_msg.elbow_angle = model->get_elbow_pose().angle;
    ee_pose_pub_->publish(ee_pose_msg);

    // Also publish the TF on /tf
    T_world_ee.header.stamp = now;
    T_world_ee.header.frame_id = "stand";
    T_world_ee.child_frame_id = "end_effector";
    tf_broadcaster_->sendTransform(T_world_ee);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_node()->get_logger(), "TF lookup world->ee failed: %s", ex.what());
  }

  return controller_interface::return_type::OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn OptimoEffortController::on_configure(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn OptimoEffortController::on_init()
{
  // This is effectively a constructor...

  LOG_INFO("OptimoEffortController::on_init: Effort controller starting...");

  // Create service for setting callbacks
  cb_group = get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
  exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec->add_callback_group(cb_group, get_node()->get_node_base_interface());
  using namespace std::placeholders;
  free_motion_srv = get_node()->create_service<optimo_msgs::srv::GenericCb>(
    "~/free_motion_cb", std::bind(&OptimoEffortController::free_motion_cb, this, _1, _2),
    rmw_qos_profile_default, cb_group);
  move_home_srv = get_node()->create_service<optimo_msgs::srv::PosCb>(
    "~/move_home_cb", std::bind(&OptimoEffortController::move_home_cb, this, _1, _2),
    rmw_qos_profile_default, cb_group);
  stop_srv = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/stop_cb", std::bind(&OptimoEffortController::stop_cb, this, _1, _2), rmw_qos_profile_default,
    cb_group);
  teach_traj_srv = get_node()->create_service<optimo_msgs::srv::StringCb>(
    "~/teach_traj_cb", std::bind(&OptimoEffortController::teach_traj_cb, this, _1, _2),
    rmw_qos_profile_default, cb_group);
  play_traj_srv = get_node()->create_service<optimo_msgs::srv::PlayTrajCb>(
    "~/play_traj_cb", std::bind(&OptimoEffortController::play_traj_cb, this, _1, _2),
    rmw_qos_profile_default, cb_group);
  moveit_srv = get_node()->create_service<optimo_msgs::srv::MoveitCb>(
    "~/moveit_cb", std::bind(&OptimoEffortController::moveit_cb, this, _1, _2),
    rmw_qos_profile_default, cb_group);
  moveit_servo_srv = get_node()->create_service<optimo_msgs::srv::GenericCb>(
    "~/moveit_servo_cb", std::bind(&OptimoEffortController::moveit_servo_cb, this, _1, _2),
    rmw_qos_profile_default, cb_group);
  servo_srv = get_node()->create_service<optimo_msgs::srv::ServoCb>(
    "~/servo_cb", std::bind(&OptimoEffortController::servo_cb, this, _1, _2),
    rmw_qos_profile_default, cb_group);
  servo_fb_srv = get_node()->create_service<optimo_msgs::srv::StringCb>(
    "~/servo_fb_cb", std::bind(&OptimoEffortController::servo_fb_cb, this, _1, _2),
    rmw_qos_profile_default, cb_group);
  enable_load_calculation_srv = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/calculate_load",
    std::bind(&OptimoEffortController::enable_load_calculation_cb, this, _1, _2),
    rmw_qos_profile_default, cb_group);

  exec_thread = std::thread([&]() { exec->spin(); });

  std::shared_ptr<roboligent::RobotConfiguration> rconfig;

  robot_prefix = get_node()->get_parameter("robot_prefix").as_string();
  robot_index = get_node()->get_parameter("robot_index").as_int();

  roboligent::Logger::Override(
    roboligent::Logger::INFO, "./log/effort_controller_" + std::to_string(robot_index) + ".log");

  if (get_node()->get_parameter("use_sim_hardware").as_bool()) {
    if (robot_prefix == "left_arm_") {
      rconfig = std::make_shared<roboligent::RobotConfiguration>(
        ament_index_cpp::get_package_share_directory("optimo_sim") +
          "/config/OR7_sim_config_mount_left.yml",
        "right_arm");
    } else if (robot_prefix == "right_arm_") {
      rconfig = std::make_shared<roboligent::RobotConfiguration>(
        ament_index_cpp::get_package_share_directory("optimo_sim") +
          "/config/OR7_sim_config_mount_right.yml",
        "right_arm");
    } else {
      rconfig = std::make_shared<roboligent::RobotConfiguration>(
        ament_index_cpp::get_package_share_directory("optimo_sim") + "/config/OR7_sim_config.yml",
        "right_arm");
    }
  } else {
    if (robot_index == 0 || robot_index == 1)
      rconfig = std::make_shared<roboligent::RobotConfiguration>(
        optimo::RSRC_PATH() + "/master" + std::to_string(robot_index) + "/OR7_config.yml",
        "right_arm");  // Fix the robot name always being right arm.
    else {
      LOG_ERROR("OptimoEffortController::on_init: EtherCAT master index invalid!");
      return CallbackReturn::ERROR;
    }
  }

  model = std::make_shared<roboligent::Model>(rconfig);
  pos = std::vector<double>(model->size(), 0);
  vel = std::vector<double>(model->size(), 0);
  eff = std::vector<int>(model->size(), 0);
  first_run = true;
  moveit_iface = std::make_shared<MoveItInterface>(*model, get_node()->get_namespace());
  torque_command = std::vector<int>(model->size(), 0);

  cb_list.emplace(
    static_cast<int>(ROSExtendedCommandID::NONE), std::make_unique<optimo::IdleCallback>(*model));
  cb_list.emplace(
    static_cast<int>(ROSExtendedCommandID::FREE_MOTION),
    std::make_unique<optimo::FreeMotionCallback>(*model));
  cb_list.emplace(
    static_cast<int>(ROSExtendedCommandID::PLAY_TRAJ),
    std::make_unique<optimo::PlayTrajCallback>(*model, current_task));
  cb_list.emplace(
    static_cast<int>(ROSExtendedCommandID::TEACH),
    std::make_unique<optimo::TeachCallback>(*model, current_task));
  cb_list.emplace(
    static_cast<int>(ROSExtendedCommandID::MOVE_HOME),
    std::make_unique<optimo::MoveHomeCallback>(*model, current_task));
  cb_list.emplace(
    static_cast<int>(ROSExtendedCommandID::MOVEIT),
    std::make_unique<MoveItCallback>(*model, current_task, *moveit_iface));
  cb_list.emplace(
    static_cast<int>(ROSExtendedCommandID::MOVEIT_SERVO),
    std::make_unique<MoveItServoCallback>(*model, current_task, *moveit_iface));
  cb_list.emplace(
    static_cast<int>(ROSExtendedCommandID::SERVO),
    std::make_unique<ServoCallback>(*model, current_task, get_node()));
  cb_list.emplace(
    static_cast<int>(ROSExtendedCommandID::SERVO_FB),
    std::make_unique<ServoFbCallback>(*model, current_task, get_node()));
  LOG_INFO("OptimoEffortController::on_init: Effort controller initialized.");

  ee_pose_pub_ =
    get_node()->create_publisher<optimo_msgs::msg::PoseElbow>("/optimo/ee_pose_current", 10);

  jacobian_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/optimo/jacobian", rclcpp::QoS(10));

  ext_wrench_pub_ =
    get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("/optimo/external_wrench", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, get_node());

  return CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::update_task()
{
  task_mtx.lock();
  if (current_task) {
    if (current_task->is_completed()) {
      // RCLCPP_INFO(get_node()->get_logger(), "Task completed, ID: %d",
      // current_task->command().id);
      complete_task();
    } else {
      // RCLCPP_INFO(get_node()->get_logger(), "Task still in progress, ID: %d",
      // current_task->command().id);
      task_mtx.unlock();
      return;
    }
  } else {
    // RCLCPP_INFO(get_node()->get_logger(), "No current task");
  }
  task_mtx.unlock();

  if (task_queue.empty()) {
    // RCLCPP_INFO(get_node()->get_logger(), "Task queue is empty");
    return;
  }

  task_mtx.lock();
  current_task = task_queue.front();
  RCLCPP_INFO(get_node()->get_logger(), "New task from queue, ID: %d", current_task->command().id);

  // start the task
  switch (current_task->command().id) {
    case static_cast<int>(ROSExtendedCommandID::PLAY_TRAJ):
    case static_cast<int>(ROSExtendedCommandID::TEACH):
    case static_cast<int>(ROSExtendedCommandID::MOVE_HOME):
    case static_cast<int>(ROSExtendedCommandID::FREE_MOTION):
    case static_cast<int>(ROSExtendedCommandID::MOVEIT):
    case static_cast<int>(ROSExtendedCommandID::MOVEIT_SERVO):
    case static_cast<int>(ROSExtendedCommandID::SERVO):
    case static_cast<int>(ROSExtendedCommandID::SERVO_FB):

      current_task->start_task();
      // RCLCPP_INFO(
      //   get_node()->get_logger(),
      //   "OptimoEffortController::update_task: Starting CommandID %d",
      //   current_task->command().id);
      break;
    default:
      RCLCPP_ERROR(
        get_node()->get_logger(), "OptimoEffortController::update_task: Unknown CommandID %d",
        current_task->command().id);
      current_task->set_status(roboligent::CommandStatus::FAILURE);
      complete_task();
      break;
  }
  task_mtx.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::complete_task()
{
  if (current_task->get_status() == roboligent::CommandStatus::FAILURE)
    LOG_INFO("OptimoEffortController::complete_task: Task failed, moving to idle callback.");
  else
    LOG_INFO("OptimoEffortController::complete_task: Task succeeded, moving to idle callback.");

  if (!task_queue.empty()) task_queue.pop();
  current_task = nullptr;

  // Ensure fallback doesn't jump and all safeties are at default values
  // This should probably become a helper in model
  model->reset_fallback();
  model->enable_fallback_defaults(true);
  model->enable_fallback_dragging(true);
  model->enable_elbow_fallback_dragging(true);

  model->enable_j1j7_sg_compensation(false);
  model->enable_j2_sg_compensation(false);
  model->enable_j4_sg_compensation(false);
  model->enable_j6_sg_compensation(false);
  model->set_angle_j6_sg_compensation(0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::free_motion_cb(
  const std::shared_ptr<optimo_msgs::srv::GenericCb::Request> request,
  const std::shared_ptr<optimo_msgs::srv::GenericCb::Response> response)
{
  roboligent::RobotCommand cmd;
  cmd.id = static_cast<int>(ROSExtendedCommandID::FREE_MOTION);
  LOG_INFO(
    "OptimoEffortController::free_motion_cb: Queued free motion (gravity compensation only).");
  response->success = queue_task(MAKE_SHARED_TASK(
    optimo::Task(cmd, request->duration > 0 ? request->duration : roboligent::Timer::NO_TIMEOUT)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::move_home_cb(
  const std::shared_ptr<optimo_msgs::srv::PosCb::Request> request,
  const std::shared_ptr<optimo_msgs::srv::PosCb::Response> response)
{
  roboligent::RobotCommand cmd;
  cmd.id = static_cast<int>(ROSExtendedCommandID::MOVE_HOME);
  cmd.command_data = std::vector<double>(request->pos);  // fix syntax

  LOG_INFO("OptimoEffortController::move_home_cb: Queued move home");
  response->success = queue_task(MAKE_SHARED_TASK(
    optimo::Task(cmd, request->duration > 0 ? request->duration : roboligent::Timer::NO_TIMEOUT)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::stop_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  task_mtx.lock();
  if (current_task) {
    // TOOO: Make it possible to only stop the current task, and still allow the rest of the
    // callbacks to be performed.
    optimo::TaskQueue empty;
    std::swap(task_queue, empty);

    // fail the current task, if it exists.
    current_task->set_status(roboligent::CommandStatus::FAILURE);
    LOG_INFO("OptimoEffortController::stop_cb: Callbacks stopped.");
    response->success = true;
  } else {
    LOG_INFO("OptimoEffortController::stop_cb: No callback to stop.");
    response->success = true;
    response->message = "No callback to stop.";
  }
  int ethercat_index = get_node()->get_parameter("robot_index").as_int();
  LOG_INFO("Stopped EtherCAT master index: " + std::to_string(ethercat_index));
  task_mtx.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::teach_traj_cb(
  const std::shared_ptr<optimo_msgs::srv::StringCb::Request> request,
  const std::shared_ptr<optimo_msgs::srv::StringCb::Response> response)
{
  // If the current command is to teach, instead end the teach callback.
  task_mtx.lock();
  // TODO figure out how to do teach_traj_cb with queue_task
  //   If the current command is to teach, instead end the teach callback.
  if (current_task && current_task->command().id == static_cast<int>(ROSExtendedCommandID::TEACH)) {
    current_task->command().data[0] = 1;
    response->success = true;
    task_mtx.unlock();
    return;
  }
  task_mtx.unlock();

  // check if request has a string
  if (request->string == "") {
    LOG_ERROR("OptimoEffortController::teach_traj_cb: No filepath given for teach command.");
    response->success = false;
    return;
  }
  roboligent::RobotCommand cmd;
  cmd.id = static_cast<int>(ROSExtendedCommandID::TEACH);
  std::copy_n(
    request->string.begin(), std::min(request->string.size(), cmd.text.size() - 1),
    cmd.text.begin());
  task_queue.push(MAKE_SHARED_TASK(
    optimo::Task(cmd, request->duration > 0 ? request->duration : roboligent::Timer::NO_TIMEOUT)));
  LOG_INFO("OptimoEffortController:teach_traj_cb: Queued trajectory teaching.");
  response->success = true;
  return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::play_traj_cb(
  const std::shared_ptr<optimo_msgs::srv::PlayTrajCb::Request> request,
  const std::shared_ptr<optimo_msgs::srv::PlayTrajCb::Response> response)
{
  if (request->goal.filepath == "") {
    // todo enable parsing of non filepath trajectories
    LOG_ERROR(
      "OptimoEffortController::play_traj_cb: No filepath given for play trajectory "
      "command.");
    response->success = false;
    return;
  }

  roboligent::RobotCommand cmd;
  cmd.id = static_cast<int>(ROSExtendedCommandID::PLAY_TRAJ);
  // transfer TrajectoryGoal
  roboligent::TrajectoryGoal goal;
  if (!goal.traj_reader.reset(request->goal.filepath)) {
    response->success = false;
    return;
  }
  if (request->goal.param.imp.impedance != 0)
    goal.param.impedance = request->goal.param.imp.impedance;
  if (request->goal.param.imp.max_force != 0)
    goal.param.max_force = request->goal.param.imp.max_force;
  if (request->goal.param.imp.stop_sensitivity != 0)
    goal.param.stop_sensitivity = request->goal.param.imp.stop_sensitivity;
  if (request->goal.param.recovery_speed != 0)
    goal.param.recovery_speed = request->goal.param.recovery_speed;
  goal.param.accept_diff_start = request->goal.param.accept_diff_start;
  goal.param.servo = false;  // should not be a parameter
  cmd.command_data = goal;

  LOG_INFO("OptimoEffortController:play_traj_cb: Queued trajectory playing.");
  response->success = queue_task(MAKE_SHARED_TASK(
    optimo::Task(cmd, request->duration > 0 ? request->duration : roboligent::Timer::NO_TIMEOUT)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::moveit_cb(
  const std::shared_ptr<optimo_msgs::srv::MoveitCb::Request> request,
  const std::shared_ptr<optimo_msgs::srv::MoveitCb::Response> response)
{
  roboligent::RobotCommand cmd;
  cmd.id = static_cast<int>(ROSExtendedCommandID::MOVEIT);
  // transfer TrajectoryGoal
  roboligent::TrajectoryGoal goal;
  cmd.data[0] = request->param.impedance;
  cmd.data[1] = request->param.max_force;
  cmd.data[2] = request->param.stop_sensitivity;
  cmd.command_data = roboligent::Pose(
    std::vector<double>(
      {request->target.position.x, request->target.position.y, request->target.position.z,
       request->target.orientation.x, request->target.orientation.y, request->target.orientation.z,
       request->target.orientation.w}));
  // currently do not accept orientation

  LOG_INFO("OptimoEffortController:moveit_cb: Queued playing a trajectory with MoveIt");
  response->success = queue_task(MAKE_SHARED_TASK(
    optimo::Task(cmd, request->duration > 0 ? request->duration : roboligent::Timer::NO_TIMEOUT)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::moveit_servo_cb(
  const std::shared_ptr<optimo_msgs::srv::GenericCb::Request> request,
  const std::shared_ptr<optimo_msgs::srv::GenericCb::Response> response)
{
  roboligent::RobotCommand cmd;
  cmd.id = static_cast<int>(ROSExtendedCommandID::MOVEIT_SERVO);
  LOG_INFO("OptimoEffortController::moveit_servo_cb: Queued servoing with MoveIt.");
  response->success = queue_task(MAKE_SHARED_TASK(
    optimo::Task(cmd, request->duration > 0 ? request->duration : roboligent::Timer::NO_TIMEOUT)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::servo_cb(
  const std::shared_ptr<optimo_msgs::srv::ServoCb::Request> request,
  const std::shared_ptr<optimo_msgs::srv::ServoCb::Response> response)
{
  roboligent::RobotCommand cmd;
  cmd.id = static_cast<int>(ROSExtendedCommandID::SERVO);
  std::copy_n(
    request->topic_name.begin(), std::min(request->topic_name.size(), cmd.text.size() - 1),
    cmd.text.begin());
  cmd.data[0] = request->cartesian;
  LOG_INFO(
    "OptimoEffortController::servo_cb: Queued servoing with " +
    std::string(request->cartesian ? "cartesian" : "joint state") +
    " topic: " + request->topic_name);
  response->success = queue_task(MAKE_SHARED_TASK(
    optimo::Task(cmd, request->duration > 0 ? request->duration : roboligent::Timer::NO_TIMEOUT)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::servo_fb_cb(
  const std::shared_ptr<optimo_msgs::srv::StringCb::Request> request,
  const std::shared_ptr<optimo_msgs::srv::StringCb::Response> response)
{
  roboligent::RobotCommand cmd;
  cmd.id = static_cast<int>(ROSExtendedCommandID::SERVO_FB);
  std::copy_n(
    request->string.begin(), std::min(request->string.size(), cmd.text.size() - 1),
    cmd.text.begin());
  LOG_INFO("OptimoEffortController::servo_cb: Queued servo feedback with a joint_states topic.");
  response->success = queue_task(MAKE_SHARED_TASK(
    optimo::Task(cmd, request->duration > 0 ? request->duration : roboligent::Timer::NO_TIMEOUT)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void OptimoEffortController::enable_load_calculation_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  auto future = model->calculate_load();
  future.wait();
  response->success = future.get();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool OptimoEffortController::queue_task(const std::shared_ptr<optimo::Task> task)
{
  task_mtx.lock();
  task_queue.push(task);
  task_mtx.unlock();
  while (true) {
    if (task->get_status() != roboligent::CommandStatus::QUEUED && task->is_timeout())
      task->set_status(roboligent::CommandStatus::FAILURE);
    // no logging because redundancy
    if (task->get_status() == roboligent::CommandStatus::SUCCESS)
      return true;
    else if (task->get_status() == roboligent::CommandStatus::FAILURE)
      return false;
    usleep(10000);  // 10 ms
  }
}

}  // namespace optimo_ros
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  optimo_ros::OptimoEffortController, controller_interface::ControllerInterface)
