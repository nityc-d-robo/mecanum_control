#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "drobo_interfaces/msg/md_lib_msg.hpp"

class MecanunmControl : public rclcpp::Node{
    private:
        rclcpp::Publisher<drobo_interfaces::msg::MdLibMsg>::SharedPtr _publisher;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription;
        void _topic_callback(const geometry_msgs::msg::Twist::SharedPtr _msg);
        void _moveChassis(double _xrpm, double _yrpm, double _yaw);
        void _sendPwm(uint8_t _address, uint8_t _semi_id, bool _phase, uint16_t _power);
    public:
        MecanunmControl(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
        MecanunmControl(
            const std::string& name_space,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
};
