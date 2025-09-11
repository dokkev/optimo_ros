#include "rclcpp/rclcpp.hpp"
#include "optimo_msgs/msg/pose_elbow.hpp"

class PoseElbowPublisher : public rclcpp::Node
{
public:
    PoseElbowPublisher()
        : Node("twist_converter")
    {
        // Create a publisher for PoseElbow messages
        publisher_ = this->create_publisher<optimo_msgs::msg::PoseElbow>("/optimo/ee_pose_desired", 10);

        // Set a timer to publish commands at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PoseElbowPublisher::publish_pose_elbow, this));
    }

private:
    void publish_pose_elbow()
    {
        auto message = optimo_msgs::msg::PoseElbow();
        message.pose.position.x = 0.4;
        message.pose.position.y = 0.1;
        message.pose.position.z = 0.5;

        // Set neutral orientation (Quaternion: No Rotation)
        message.pose.orientation.x = 0.0;
        message.pose.orientation.y = 0.0;
        message.pose.orientation.z = 0.0;
        message.pose.orientation.w = 1.0;

        // Set the elbow angle (example value)
        // message.elbow_angle = 0.0;

        RCLCPP_INFO(this->get_logger(), "Publishing PoseElbow: [%.2f, %.2f, %.2f], Elbow: %.2f",
                    message.pose.position.x, message.pose.position.y, message.pose.position.z,
                    message.elbow_angle);

        // Publish the message
        publisher_->publish(message);
    }

    rclcpp::Publisher<optimo_msgs::msg::PoseElbow>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseElbowPublisher>());
    rclcpp::shutdown();
    return 0;
}
