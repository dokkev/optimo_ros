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

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "optimo_api/Resource.h"
#include "rclcpp/rclcpp.hpp"
#include "rl/common/RobotConfiguration.h"
#include "rl/model/Model.h"
#include "rl/util/Math.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace optimo_state_estimator
{
using namespace std::chrono_literals;

class StateEstimatorNode : public rclcpp::Node
{
public:
  explicit StateEstimatorNode(const rclcpp::NodeOptions & options)
  : Node("optimo_state_estimator", options)
  {
    robot_prefix_ = declare_parameter<std::string>("robot_prefix", "");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    link0_frame_ = declare_parameter<std::string>("link0_frame", "link0");
    ee_frame_ = declare_parameter<std::string>("ee_frame", "ee");
    use_sim_hardware_ = declare_parameter<bool>("use_sim_hardware", true);
    robot_index_ = declare_parameter<int>("robot_index", 0);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);

    const auto publish_rate_hz = declare_parameter<double>("publish_rate", 30.0);
    tf_timeout_ = tf2::durationFromSec(declare_parameter<double>("tf_timeout", 0.05));
    publish_period_ = std::chrono::duration<double>(1.0 / publish_rate_hz);

    const std::vector<std::string> default_joints{
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    const auto joint_param =
      declare_parameter<std::vector<std::string>>("joints", default_joints);
    for (size_t idx = 0; idx < joint_param.size(); ++idx) {
      joint_index_[joint_param[idx]] = idx;
      joint_index_[robot_prefix_ + joint_param[idx]] = idx;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    base_tf_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("tf_base_to_ee", 10);
    link0_tf_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("tf_link0_to_ee", 10);
    model_ee_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("model_ee_pose", 10);
    model_tcp_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("model_tcp_pose", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

    try {
      init_model();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize Roboligent model: %s", e.what());
    }

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&StateEstimatorNode::joint_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      publish_period_, std::bind(&StateEstimatorNode::publish_estimates, this));

    RCLCPP_INFO(get_logger(), "optimo_state_estimator started.");
  }

private:
  void init_model()
  {
    std::shared_ptr<roboligent::RobotConfiguration> rconfig;
    if (use_sim_hardware_) {
      const auto share_dir = ament_index_cpp::get_package_share_directory("optimo_sim");
      if (robot_prefix_ == "left_arm_") {
        rconfig = std::make_shared<roboligent::RobotConfiguration>(
          share_dir + "/config/OR7_sim_config_mount_left.yml", "right_arm");
      } else if (robot_prefix_ == "right_arm_") {
        rconfig = std::make_shared<roboligent::RobotConfiguration>(
          share_dir + "/config/OR7_sim_config_mount_right.yml", "right_arm");
      } else {
        rconfig = std::make_shared<roboligent::RobotConfiguration>(
          share_dir + "/config/OR7_sim_config.yml", "right_arm");
      }
    } else {
      if (robot_index_ < 0 || robot_index_ > 1) {
        throw std::runtime_error("robot_index must be 0 or 1");
      }
      rconfig = std::make_shared<roboligent::RobotConfiguration>(
        optimo::RSRC_PATH() + "/master" + std::to_string(robot_index_) + "/OR7_config.yml",
        "right_arm");
    }

    model_ = std::make_shared<roboligent::Model>(rconfig);
    last_positions_deg_.assign(model_->size(), 0.0);
    last_velocities_deg_.assign(model_->size(), 0.0);
    zero_torque_.assign(model_->size(), 0);
  }

  static geometry_msgs::msg::Pose pose_from_roboligent_pose(const roboligent::Pose & pose)
  {
    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x = pose.position.x();
    ros_pose.position.y = pose.position.y();
    ros_pose.position.z = pose.position.z();

    const Eigen::Quaterniond q(pose.orientation);
    ros_pose.orientation.x = q.x();
    ros_pose.orientation.y = q.y();
    ros_pose.orientation.z = q.z();
    ros_pose.orientation.w = q.w();
    return ros_pose;
  }

  static geometry_msgs::msg::Pose pose_from_vec(const std::vector<double> & vec)
  {
    geometry_msgs::msg::Pose ros_pose;
    if (vec.size() < 7) {
      return ros_pose;
    }
    ros_pose.position.x = vec[0];
    ros_pose.position.y = vec[1];
    ros_pose.position.z = vec[2];

    Eigen::Vector3d axis(vec[3], vec[4], vec[5]);
    if (axis.isZero(1e-9)) axis = Eigen::Vector3d::UnitZ();
    axis.normalize();
    const Eigen::Quaterniond q(Eigen::AngleAxisd(vec[6], axis));
    ros_pose.orientation.x = q.x();
    ros_pose.orientation.y = q.y();
    ros_pose.orientation.z = q.z();
    ros_pose.orientation.w = q.w();
    return ros_pose;
  }

  void joint_callback(const sensor_msgs::msg::JointState & msg)
  {
    if (!model_) return;
    if (msg.position.empty()) return;

    auto positions = last_positions_deg_;
    auto velocities = last_velocities_deg_;

    for (size_t i = 0; i < msg.name.size(); ++i) {
      const auto it = joint_index_.find(msg.name[i]);
      if (it == joint_index_.end()) continue;
      const size_t idx = it->second;
      if (idx >= positions.size()) continue;

      positions[idx] = msg.position[i] * roboligent::RAD2DEG;
      if (!msg.velocity.empty() && msg.velocity.size() > i) {
        velocities[idx] = msg.velocity[i] * roboligent::RAD2DEG;
      } else {
        velocities[idx] = 0.0;
      }
    }

    last_positions_deg_ = positions;
    last_velocities_deg_ = velocities;

    model_->update(last_positions_deg_, last_velocities_deg_,
                   std::vector<double>(model_->size(), 0.0), zero_torque_);
    model_ready_ = true;
  }

  std::optional<geometry_msgs::msg::PoseStamped> lookup_pose(
    const std::string & target_frame, const std::string & source_frame)
  {
    try {
      auto transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero, tf_timeout_);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF lookup %s -> %s failed: %s", target_frame.c_str(),
        source_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  visualization_msgs::msg::Marker make_marker(
    const geometry_msgs::msg::PoseStamped & pose, int id, float r, float g, float b,
    const std::string & ns, bool text, const std::string & label)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = ns;
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::ADD;
    if (text) {
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.pose = pose.pose;
      marker.pose.position.z += 0.04;
      marker.scale.z = 0.04;
      marker.text = label;
    } else {
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.pose = pose.pose;
      marker.scale.x = 0.08;
      marker.scale.y = 0.015;
      marker.scale.z = 0.015;
    }
    marker.color.a = 0.9;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.lifetime = rclcpp::Duration(0, 0);
    return marker;
  }

  void publish_estimates()
  {
    const auto now = get_clock()->now();

    auto base_pose_opt = lookup_pose(base_frame_, ee_frame_);
    if (base_pose_opt) {
      base_pose_opt->header.stamp = now;
      base_tf_pub_->publish(*base_pose_opt);
    }

    std::optional<geometry_msgs::msg::PoseStamped> link0_in_base;
    auto link0_pose_opt = lookup_pose(link0_frame_, ee_frame_);
    if (link0_pose_opt) {
      link0_pose_opt->header.stamp = now;
      if (link0_frame_ != base_frame_) {
        try {
          auto to_base =
            tf_buffer_->lookupTransform(base_frame_, link0_frame_, tf2::TimePointZero, tf_timeout_);
          geometry_msgs::msg::PoseStamped transformed;
          tf2::doTransform(*link0_pose_opt, transformed, to_base);
          link0_in_base = transformed;
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "TF lookup %s -> %s failed: %s",
            base_frame_.c_str(), link0_frame_.c_str(), ex.what());
          link0_in_base = link0_pose_opt;
        }
      } else {
        link0_in_base = link0_pose_opt;
      }

      link0_tf_pub_->publish(*link0_pose_opt);
    }

    std::optional<geometry_msgs::msg::PoseStamped> model_ee_pose;
    std::optional<geometry_msgs::msg::PoseStamped> model_tcp_pose;
    if (model_ready_ && model_) {
      geometry_msgs::msg::PoseStamped ee_msg;
      ee_msg.header.frame_id = base_frame_;
      ee_msg.header.stamp = now;
      ee_msg.pose = pose_from_roboligent_pose(model_->get_ee_pose());
      model_ee_pub_->publish(ee_msg);
      model_ee_pose = ee_msg;

      geometry_msgs::msg::PoseStamped tcp_msg;
      tcp_msg.header.frame_id = base_frame_;
      tcp_msg.header.stamp = now;
      tcp_msg.pose = pose_from_vec(model_->get_tcp_pose());
      model_tcp_pub_->publish(tcp_msg);
      model_tcp_pose = tcp_msg;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 4000, "Waiting for joint_states before publishing model poses");
    }

    if (!publish_markers_) return;

    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    if (base_pose_opt) {
      markers.markers.push_back(
        make_marker(*base_pose_opt, ++id, 0.0f, 0.4f, 1.0f, "tf_base", false, ""));
      markers.markers.push_back(
        make_marker(*base_pose_opt, ++id, 0.0f, 0.4f, 1.0f, "tf_base_text", true, "tf base->ee"));
    }
    if (link0_in_base) {
      markers.markers.push_back(
        make_marker(*link0_in_base, ++id, 0.0f, 0.8f, 0.0f, "tf_link0", false, ""));
      markers.markers.push_back(make_marker(
        *link0_in_base, ++id, 0.0f, 0.8f, 0.0f, "tf_link0_text", true, "tf link0->ee"));
    }
    if (model_ee_pose) {
      markers.markers.push_back(
        make_marker(*model_ee_pose, ++id, 1.0f, 0.2f, 0.2f, "model_ee", false, ""));
      markers.markers.push_back(make_marker(
        *model_ee_pose, ++id, 1.0f, 0.2f, 0.2f, "model_ee_text", true, "model get_ee_pose"));
    }
    if (model_tcp_pose) {
      markers.markers.push_back(
        make_marker(*model_tcp_pose, ++id, 1.0f, 0.6f, 0.0f, "model_tcp", false, ""));
      markers.markers.push_back(make_marker(
        *model_tcp_pose, ++id, 1.0f, 0.6f, 0.0f, "model_tcp_text", true, "model get_tcp_pose"));
    }

    if (!markers.markers.empty()) marker_pub_->publish(markers);
  }

  // Parameters
  std::string robot_prefix_;
  std::string base_frame_;
  std::string link0_frame_;
  std::string ee_frame_;
  bool use_sim_hardware_{true};
  int robot_index_{0};
  bool publish_markers_{true};
  std::chrono::duration<double> publish_period_{33ms};
  tf2::Duration tf_timeout_{std::chrono::milliseconds(50)};

  // Model
  std::shared_ptr<roboligent::Model> model_;
  std::vector<double> last_positions_deg_;
  std::vector<double> last_velocities_deg_;
  std::vector<int> zero_torque_;
  bool model_ready_{false};

  // ROS
  std::unordered_map<std::string, size_t> joint_index_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_tf_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr link0_tf_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr model_ee_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr model_tcp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace optimo_state_estimator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optimo_state_estimator::StateEstimatorNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
