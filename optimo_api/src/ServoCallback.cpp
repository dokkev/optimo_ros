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

#include "optimo_api/ServoCallback.h"

namespace optimo_ros
{
std::mutex ServoCallback::trajectory_mutex_;

ServoCallback::ServoCallback(
  roboligent::Model & model_, optimo::TaskQueueObject & task_,
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_)
: roboligent::AbstractCallback(model_),
  model(model_),
  task(task_),
  node(node_),
  last_unique_msg_time(node_->now())
{
  sub_future = std::future<void>();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void ServoCallback::calculate_torque(std::vector<int> & torque_)
{
  model.enable_rapid_motion(true);
  switch (task->get_status()) {
    case roboligent::CommandStatus::IDLE: {
      RCLCPP_INFO(node->get_logger(), "ServoCallback::calculate_torque() COMMAND Status: IDLE");
      if (!sub_future.valid()) {
        // Start the subscription creation asynchronously
        sub_future = std::async(std::launch::async, &ServoCallback::create_subscription, this);
      }
      if (sub_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        if ((!joint_sub && !cartesian_sub) || !pub) {
          LOG_ERROR(
            "ServoCallback::calculate_torque: Failed to create subscription to " +
            std::string(task->command().text.data()));
          task->set_status(roboligent::CommandStatus::FAILURE);
          return;
        }
        LOG_INFO(
          "ServoCallback::calculate_torque: Subscription created to " +
          std::string(task->command().text.data()));

        if (task->command().data[0]) {
          trajectory = std::make_unique<roboligent::CartesianTrajectory>(model);
          trajectory->set_servo_filters(5, 0);
          servo_point = model.get_tcp_pose();
          last_target_pose = optimo_msgs::msg::PoseElbow();
          servo_point.push_back(model.get_elbow_pose().angle);
        } else {
          trajectory = std::make_unique<roboligent::JointTrajectory>(model);
          trajectory->set_servo_filters(5, 10);
          // reset servo point to current position
          servo_point = model.get_position();
        }

        // turn on singularity safeties
        model.enable_j1j7_sg_compensation(true);
        model.enable_j2_sg_compensation(true);
        model.enable_j4_sg_compensation(true);
        model.enable_j6_sg_compensation(true);
        model.set_angle_j6_sg_compensation(0);

        // Reset trajectory
        trajectory->stop();
        roboligent::TrajectoryGoal goal;
        goal.param.servo = true;
        goal.param.impedance = 10;
        goal.param.max_force = 10;
        goal.param.stop_sensitivity = 3;
        trajectory->start(goal);
        task->set_status(roboligent::CommandStatus::IN_PROGRESS);
      }

      break;
    }
    case roboligent::CommandStatus::IN_PROGRESS: {
      // std::lock_guard<std::mutex> lock(trajectory_mutex_);
      // FIXME: this should not require a mutex. Figure out the relationship between multiple
      // OptimoEffortControllers
      if (task->command().data[0]) convert_pose_to_servo_point();
      
      auto servo_torque = trajectory->calculate_servo_torque(servo_point);
      // print servo point


      auto message = std_msgs::msg::Float64MultiArray();
      for (int i = 0; i < servo_torque.size(); ++i) message.data.push_back(servo_torque[i]);
      pub->publish(message);

      torque_ = torque_ + servo_torque;
      break;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void ServoCallback::create_subscription()
{
  // Clean up existing subscriptions first
  if (cartesian_sub) {
    cartesian_sub.reset();
  }
  if (joint_sub) {
    joint_sub.reset();
  }

  auto sub_name = std::string(task->command().text.data());

  if (task->command().data[0]) {
    cartesian_sub = node->create_subscription<optimo_msgs::msg::PoseElbow>(
      sub_name, 10, std::bind(&ServoCallback::cartesian_cb, this, std::placeholders::_1));
  } else {
    joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      sub_name, 10, std::bind(&ServoCallback::joint_cb, this, std::placeholders::_1));
  }
  pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    std::string(node->get_namespace()) + "/servo_fb", 10);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void ServoCallback::cartesian_cb(const optimo_msgs::msg::PoseElbow & msg)
{
  total_messages++;

  bool is_different = (msg.pose.position.x != last_target_pose.pose.position.x) ||
                      (msg.pose.position.y != last_target_pose.pose.position.y) ||
                      (msg.pose.position.z != last_target_pose.pose.position.z) ||
                      (msg.pose.orientation.x != last_target_pose.pose.orientation.x) ||
                      (msg.pose.orientation.y != last_target_pose.pose.orientation.y) ||
                      (msg.pose.orientation.z != last_target_pose.pose.orientation.z) ||
                      (msg.pose.orientation.w != last_target_pose.pose.orientation.w) ||
                      (msg.elbow_angle != last_target_pose.elbow_angle);

  if (is_different) {
    rclcpp::Time current_time = node->now();
    int64_t time_diff_ms = (current_time - last_unique_msg_time).nanoseconds() / 1000000;

    if (time_diff_ms > 200) {
      RCLCPP_WARN(
        node->get_logger(),
        "ServoCallback::cartesian_cb: %ld ms since last unique message (message %d)", time_diff_ms,
        total_messages);
    }

    last_unique_msg_time = current_time;
    last_target_pose = msg;
  }
}

void ServoCallback::convert_pose_to_servo_point()
{
  if (
    last_target_pose.pose.position.x == 0 && last_target_pose.pose.position.y == 0 &&
    last_target_pose.pose.position.z == 0)
    return;  // should check if the last_target_pose is valid
  servo_point[0] = last_target_pose.pose.position.x;
  servo_point[1] = last_target_pose.pose.position.y;
  servo_point[2] = last_target_pose.pose.position.z;

  // need to convert the orientation from quaternion to our weird custom angle representation.
  // Ours defines the first 3 spots as the z axis of the frame, and the last as the angle on the
  // joint corresponding to the frame.

  // First, convert quaternion to rotation matrix
  Eigen::Quaterniond q(
    last_target_pose.pose.orientation.w, last_target_pose.pose.orientation.x,
    last_target_pose.pose.orientation.y, last_target_pose.pose.orientation.z);
  Eigen::Matrix3d R = q.toRotationMatrix();

  // Extract the x and z axes from the rotation matrix
  Eigen::Vector3d target_x_axis = R.col(0).normalized();
  Eigen::Vector3d target_z_axis = R.col(2).normalized();

  servo_point[3] = target_z_axis(0);
  servo_point[4] = target_z_axis(1);
  servo_point[5] = target_z_axis(2);

  // For the angle, we use J6's current frame, get the x axis, project it onto the xy plane of R,
  // then find the angle between the projection and  x axis of R.
  Eigen::Vector3d J6_x_axis = model.get_rotation_eigen(6).col(0);
  Eigen::Vector3d proj = (J6_x_axis - target_z_axis * target_z_axis.dot(J6_x_axis)).normalized();
  double angle = roboligent::angle_between_vectors(proj, target_x_axis);
  // angle_between_vectors does not actually produce negative angles.
  if (target_x_axis.cross(proj).dot(target_z_axis) > 0) angle = -angle;
  servo_point[6] = angle;
  servo_point[7] = last_target_pose.elbow_angle;

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void ServoCallback::joint_cb(const sensor_msgs::msg::JointState & msg)
{
  // get position information from the ros_msg
  std::vector<double> servo_point_(model.size(), 0);
  for (int i = 0; i < msg.name.size(); i++) {
    // ROS names the joints from 1-7, and pushes them back in a scrambled order
    if (msg.name[i].back() == 't') continue;
    int joint_num = static_cast<int>(msg.name[i].back()) - '0' - 1;
    if (std::isnan(msg.position[i])) return;
    servo_point_[joint_num] = (msg.position[i]);
  }
  // this part works similar to joint fallback's anchoring effect
  // no filtering is done here because topic callback can run in erratic rhythm.
  for (int i = 0; i < model.size(); ++i) servo_point[i] = servo_point_[i];
}

}  // namespace optimo_ros
