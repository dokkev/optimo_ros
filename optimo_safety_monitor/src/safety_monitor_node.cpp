#include "optimo_safety_monitor/safety_monitor_node.hpp"

#include <cmath>
#include <chrono>

SafetyMonitorNode::SafetyMonitorNode(const rclcpp::NodeOptions & options)
: Node("safety_monitor", options)
{
  declare_parameter<double>("check_rate_hz", 50.0);
  declare_parameter<double>("force_threshold", 30.0);
  declare_parameter<double>("effort_threshold", 5.0);

  force_threshold_ = get_parameter("force_threshold").as_double();
  effort_threshold_ = get_parameter("effort_threshold").as_double();
  double check_rate_hz = get_parameter("check_rate_hz").as_double();

  // Subscribers
  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/optimo/external_wrench", 10,
    std::bind(&SafetyMonitorNode::wrench_callback, this, std::placeholders::_1));

  pose_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/optimo/human_pose", 10,
    std::bind(&SafetyMonitorNode::pose_callback, this, std::placeholders::_1));

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/optimo/joint_states", 10,
    std::bind(&SafetyMonitorNode::joint_state_callback, this, std::placeholders::_1));

  // Publishers
  status_pub_ = create_publisher<std_msgs::msg::Bool>("~/status", 10);
  effort_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/effort_debug", 10);

  // Service client
  stop_client_ = create_client<std_srvs::srv::Trigger>(
    "/optimo/optimo_effort_controller/stop_cb");

  // Timer
  auto period = std::chrono::duration<double>(1.0 / check_rate_hz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SafetyMonitorNode::timer_callback, this));

  RCLCPP_INFO(get_logger(),
    "Safety monitor started (rate=%.1f Hz, effort_threshold=%.1f)",
    check_rate_hz, effort_threshold_);
}

void SafetyMonitorNode::wrench_callback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_wrench_ = *msg;
  wrench_received_ = true;
}

void SafetyMonitorNode::pose_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_pose_ = *msg;
  pose_received_ = true;
}

void SafetyMonitorNode::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_joint_state_ = *msg;
  joint_state_received_ = true;
}

bool SafetyMonitorNode::is_force_safe()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!joint_state_received_) {
    return true;
  }

  // Check joint1 effort only
  const auto & names = latest_joint_state_.name;
  const auto & efforts = latest_joint_state_.effort;

  for (size_t i = 0; i < names.size() && i < efforts.size(); ++i) {
    if (names[i] == "joint1") {
      double effort = std::abs(efforts[i]);
      if (effort > effort_threshold_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "joint1 effort %.2f exceeds threshold %.2f", effort, effort_threshold_);
        return false;
      }
      break;
    }
  }

  return true;
}

bool SafetyMonitorNode::is_pose_safe()
{
  // TODO: Replace with real human pose estimation safety logic.
  // Example: check if any human joint is within a danger zone around the robot
  //
  // std::lock_guard<std::mutex> lock(mutex_);
  // if (!pose_received_) return true;
  // for (size_t i = 0; i < latest_pose_.position.size(); i += 3) {
  //   double dist = compute_distance_to_robot(
  //     latest_pose_.position[i],
  //     latest_pose_.position[i+1],
  //     latest_pose_.position[i+2]);
  //   if (dist < safety_radius_) return false;
  // }

  return true;
}

void SafetyMonitorNode::trigger_stop()
{
  if (!stop_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "stop_cb service not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  stop_client_->async_send_request(request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto result = future.get();
      if (result->success) {
        RCLCPP_WARN(get_logger(), "Stop triggered successfully");
      } else {
        RCLCPP_ERROR(get_logger(), "Stop call failed: %s", result->message.c_str());
      }
    });
}

void SafetyMonitorNode::timer_callback()
{
  bool safe = is_force_safe() && is_pose_safe();
  auto stamp = now();

  std_msgs::msg::Bool status_msg;
  status_msg.data = safe;
  status_pub_->publish(status_msg);

  // Publish effort debug topic
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (joint_state_received_) {
      sensor_msgs::msg::JointState effort_msg;
      effort_msg.header.stamp = stamp;
      effort_msg.name = latest_joint_state_.name;
      effort_msg.effort = latest_joint_state_.effort;
      effort_pub_->publish(effort_msg);
    }
  }

  // Periodically log max effort for visibility
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (joint_state_received_) {
      const auto & names = latest_joint_state_.name;
      const auto & efforts = latest_joint_state_.effort;
      double max_effort = 0.0;
      std::string max_name;
      for (size_t i = 0; i < names.size() && i < efforts.size(); ++i) {
        if (std::abs(efforts[i]) > max_effort) {
          max_effort = std::abs(efforts[i]);
          max_name = names[i];
        }
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%.3f] max effort: %s=%.2f (threshold=%.2f) status=%s",
        stamp.seconds(), max_name.c_str(), max_effort, effort_threshold_,
        safe ? "SAFE" : "UNSAFE");
    }
  }

  if (!safe && !stop_triggered_) {
    RCLCPP_ERROR(get_logger(), "[%.3f] UNSAFE — STOP", stamp.seconds());
    trigger_stop();
    stop_triggered_ = true;
  }

  // Reset flag when safe again so we can trigger stop on the next danger event
  if (safe && stop_triggered_) {
    RCLCPP_INFO(get_logger(), "[%.3f] Safety restored", stamp.seconds());
    stop_triggered_ = false;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
