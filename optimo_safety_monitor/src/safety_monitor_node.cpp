#include "optimo_safety_monitor/safety_monitor_node.hpp"

#include <cmath>
#include <chrono>
#include <fstream>
#include <sstream>
#include <limits>

SafetyMonitorNode::SafetyMonitorNode(const rclcpp::NodeOptions & options)
: Node("safety_monitor", options)
{
  declare_parameter<double>("check_rate_hz", 50.0);
  declare_parameter<double>("wrench_force_threshold", 10.0);
  declare_parameter<std::string>("baseline_file", "");

  wrench_force_threshold_ = get_parameter("wrench_force_threshold").as_double();
  baseline_file_ = get_parameter("baseline_file").as_string();
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

  // Service client
  stop_client_ = create_client<std_srvs::srv::Trigger>(
    "/optimo/optimo_effort_controller/stop_cb");

  // Baseline services
  record_baseline_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/record_baseline",
    std::bind(&SafetyMonitorNode::record_baseline_cb, this,
      std::placeholders::_1, std::placeholders::_2));

  save_baseline_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/save_baseline",
    std::bind(&SafetyMonitorNode::save_baseline_cb, this,
      std::placeholders::_1, std::placeholders::_2));

  // Load baseline if file provided
  if (!baseline_file_.empty()) {
    if (load_baseline(baseline_file_)) {
      RCLCPP_INFO(get_logger(), "Loaded %zu baseline samples from %s",
        baseline_data_.size(), baseline_file_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to load baseline from %s", baseline_file_.c_str());
    }
  }

  // Timer
  auto period = std::chrono::duration<double>(1.0 / check_rate_hz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SafetyMonitorNode::timer_callback, this));

  RCLCPP_INFO(get_logger(),
    "Safety monitor started (rate=%.1f Hz, wrench_threshold=%.1f)",
    check_rate_hz, wrench_force_threshold_);
}

void SafetyMonitorNode::wrench_callback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_wrench_ = *msg;
  wrench_received_ = true;

  // Record baseline sample if recording
  if (recording_ && joint_state_received_) {
    BaselineSample sample;
    sample.joint_pos = latest_joint_state_.position;
    sample.fx = msg->wrench.force.x;
    sample.fy = msg->wrench.force.y;
    sample.fz = msg->wrench.force.z;
    baseline_data_.push_back(sample);
  }
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

bool SafetyMonitorNode::is_pose_safe()
{
  return true;
}

bool SafetyMonitorNode::is_wrench_safe()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!baseline_loaded_ || !wrench_received_ || !joint_state_received_) {
    return true;
  }

  auto baseline = find_nearest_baseline(latest_joint_state_.position);

  double dx = latest_wrench_.wrench.force.x - baseline.fx;
  double dy = latest_wrench_.wrench.force.y - baseline.fy;
  double dz = latest_wrench_.wrench.force.z - baseline.fz;
  double delta_mag = std::sqrt(dx * dx + dy * dy + dz * dz);
  last_delta_force_ = delta_mag;

  if (delta_mag > wrench_force_threshold_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
      "External force detected: delta=%.2f N (threshold=%.2f) [dx=%.2f dy=%.2f dz=%.2f]",
      delta_mag, wrench_force_threshold_, dx, dy, dz);
    return false;
  }

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

// --- Baseline services ---

void SafetyMonitorNode::record_baseline_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  baseline_data_.clear();
  recording_ = true;
  baseline_loaded_ = false;
  response->success = true;
  response->message = "Recording baseline. Play trajectory now, then call save_baseline.";
  RCLCPP_INFO(get_logger(), "Baseline recording started");
}

void SafetyMonitorNode::save_baseline_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  recording_ = false;

  if (baseline_data_.empty()) {
    response->success = false;
    response->message = "No baseline data recorded.";
    return;
  }

  if (baseline_file_.empty()) {
    baseline_file_ = "/tmp/optimo_wrench_baseline.csv";
  }

  std::ofstream file(baseline_file_);
  if (!file.is_open()) {
    response->success = false;
    response->message = "Failed to open file: " + baseline_file_;
    return;
  }

  // Header
  file << "q1,q2,q3,q4,q5,q6,q7,fx,fy,fz\n";
  for (const auto & s : baseline_data_) {
    for (size_t i = 0; i < s.joint_pos.size(); ++i) {
      file << s.joint_pos[i];
      file << ",";
    }
    file << s.fx << "," << s.fy << "," << s.fz << "\n";
  }
  file.close();

  baseline_loaded_ = true;
  response->success = true;
  response->message = "Saved " + std::to_string(baseline_data_.size()) +
    " samples to " + baseline_file_;
  RCLCPP_INFO(get_logger(), "Baseline saved: %zu samples to %s",
    baseline_data_.size(), baseline_file_.c_str());
}

bool SafetyMonitorNode::load_baseline(const std::string & filepath)
{
  std::ifstream file(filepath);
  if (!file.is_open()) {
    return false;
  }

  baseline_data_.clear();
  std::string line;

  // Skip header
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string token;
    BaselineSample sample;

    // Read 7 joint positions
    for (int i = 0; i < 7; ++i) {
      if (!std::getline(ss, token, ',')) return false;
      sample.joint_pos.push_back(std::stod(token));
    }

    // Read fx, fy, fz
    if (!std::getline(ss, token, ',')) return false;
    sample.fx = std::stod(token);
    if (!std::getline(ss, token, ',')) return false;
    sample.fy = std::stod(token);
    if (!std::getline(ss, token, ',')) return false;
    sample.fz = std::stod(token);

    baseline_data_.push_back(sample);
  }

  baseline_loaded_ = !baseline_data_.empty();
  return baseline_loaded_;
}

BaselineSample SafetyMonitorNode::find_nearest_baseline(const std::vector<double> & joint_pos)
{
  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < baseline_data_.size(); ++i) {
    const auto & bp = baseline_data_[i].joint_pos;
    double dist = 0.0;
    size_t n = std::min(joint_pos.size(), bp.size());
    for (size_t j = 0; j < n; ++j) {
      double d = joint_pos[j] - bp[j];
      dist += d * d;
    }
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  return baseline_data_[min_idx];
}

void SafetyMonitorNode::timer_callback()
{
  bool safe = is_pose_safe() && is_wrench_safe();
  auto stamp = now();

  std_msgs::msg::Bool status_msg;
  status_msg.data = safe;
  status_pub_->publish(status_msg);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "[%.3f] baseline=%s status=%s delta_force=%.2f N",
    stamp.seconds(),
    baseline_loaded_ ? "loaded" : "none", safe ? "SAFE" : "UNSAFE",
    last_delta_force_);

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
