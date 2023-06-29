#include <string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MecanunmControl : public rclcpp::Node{
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription;
        void _topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    public:
        MecanunmControl(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
        MecanunmControl(
            const std::string& name_space,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
};