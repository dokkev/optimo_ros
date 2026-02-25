#ifndef OPTIMO_SAFETY_MONITOR__SAFETY_MONITOR_NODE_HPP_
#define OPTIMO_SAFETY_MONITOR__SAFETY_MONITOR_NODE_HPP_

#include <mutex>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

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

  bool is_force_safe();

  // TODO: Implement real pose safety logic (evaluate human skeleton proximity/danger)
  bool is_pose_safe();

  void trigger_stop();

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr effort_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::TimerBase::SharedPtr timer_;

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

  // Parameters
  double force_threshold_;
  double effort_threshold_;
};

#endif  // OPTIMO_SAFETY_MONITOR__SAFETY_MONITOR_NODE_HPP_
