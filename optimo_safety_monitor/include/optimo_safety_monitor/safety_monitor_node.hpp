#ifndef OPTIMO_SAFETY_MONITOR__SAFETY_MONITOR_NODE_HPP_
#define OPTIMO_SAFETY_MONITOR__SAFETY_MONITOR_NODE_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

struct BaselineSample {
  std::vector<double> joint_pos;  // 7 joint angles
  double fx, fy, fz;             // force at this configuration
};

struct JointAngleRule {
  std::string name;
  int joint_a, joint_b, joint_c;  // angle measured at joint_b
  double min_deg, max_deg;
};

struct TorsoAngleRule {
  std::string name;
  int axis1_a, axis1_b;  // first axis (e.g. shoulder line)
  int axis2_a, axis2_b;  // second axis (e.g. hip line)
  double min_deg, max_deg;
};

class SafetyMonitorNode : public rclcpp::Node
{
public:
  explicit SafetyMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SafetyMonitorNode() = default;

private:
  void timer_callback();

  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void pose_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  bool is_pose_safe();
  bool is_wrench_safe();

  void trigger_stop();

  // Baseline methods
  void record_baseline_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void save_baseline_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool load_baseline(const std::string & filepath);
  BaselineSample find_nearest_baseline(const std::vector<double> & joint_pos);

  // Pose rule methods
  bool load_pose_rules(const std::string & filepath);
  static double compute_angle(const std::vector<double> & pos, int a, int b, int c);
  static double compute_axis_angle(const std::vector<double> & pos,
    int a1, int b1, int a2, int b2);

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr human_pose_safe_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Baseline services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr record_baseline_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_baseline_srv_;

  // Latest sensor data (guarded by mutex_)
  std::mutex mutex_;
  geometry_msgs::msg::WrenchStamped latest_wrench_;
  sensor_msgs::msg::JointState latest_pose_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool wrench_received_{false};
  bool pose_received_{false};
  bool joint_state_received_{false};

  // State
  bool stop_triggered_{false};

  // Baseline state
  bool recording_{false};
  bool baseline_loaded_{false};
  std::vector<BaselineSample> baseline_data_;
  std::string baseline_file_;

  // Pose rule state
  bool pose_rules_loaded_{false};
  std::vector<JointAngleRule> joint_angle_rules_;
  std::vector<TorsoAngleRule> torso_angle_rules_;
  std::string pose_rules_file_;
  double pose_angle_margin_deg_{10.0};

  // Parameters
  double wrench_force_threshold_;
  double last_delta_force_{0.0};
};

#endif  // OPTIMO_SAFETY_MONITOR__SAFETY_MONITOR_NODE_HPP_
