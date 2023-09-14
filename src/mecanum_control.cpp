#include "mecanum_control/mecanum_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <cstdio>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include "drobo_interfaces/msg/md_lib_msg.hpp"

constexpr double MECANUNM_DIA = 0.152;
constexpr double ROBOT_CENTER_TO_WHEEL_DISTANCE = 0.37; // ロボットの重心からメカナムホイールまでの距離

void MecanunmControl::_sendPwm(uint8_t _address, uint8_t _semi_id, bool _phase, uint16_t _power){
    auto msg = std::make_shared<drobo_interfaces::msg::MdLibMsg>();
    msg->address = _address;
    msg->semi_id = _semi_id;
    msg->mode = 2; //MotorLibのPWMモードに倣いました
    msg->phase = _phase;
    msg->power = _power;
    _publisher->publish(*msg);
}

void MecanunmControl::_moveChassis(double _xrpm, double _yrpm, double _yaw) {
    /// TODO:制御式落とし込んでsendSpeed()する
    double speed_abs = sqrt(pow(_xrpm, 2) + pow(_yrpm, 2));
    double radwimps = atan2(_yrpm, _xrpm);

    int wheel1_rpm = ((speed_abs * sin(radwimps)) + (speed_abs * cos(radwimps)) + (2*sqrt(2)*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4 * M_PI * MECANUNM_DIA;
    int wheel2_rpm = ((speed_abs * sin(radwimps)) - (speed_abs * cos(radwimps)) - (2*sqrt(2)*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4 * M_PI * MECANUNM_DIA;
    int wheel3_rpm = ((speed_abs * sin(radwimps)) + (speed_abs * cos(radwimps)) - (2*sqrt(2)*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4 * M_PI * MECANUNM_DIA;
    int wheel4_rpm = ((speed_abs * sin(radwimps)) - (speed_abs * cos(radwimps)) + (2*sqrt(2)*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4 * M_PI * MECANUNM_DIA;
    
    RCLCPP_INFO(this->get_logger(), "wheel1: %lf, wheel2: %lf, wheel3: %lf, wheel4: %lf\n", wheel1_rpm, wheel2_rpm, wheel3_rpm, wheel4_rpm);

    _sendPwm(0x00, NULL, wheel1_rpm >= 0 ? false : true, std::clamp(abs(wheel1_rpm), 0, 999));
    _sendPwm(0x01, NULL, wheel2_rpm >= 0 ? true : false, std::clamp(abs(wheel2_rpm), 0, 999));
    _sendPwm(0x02, NULL, wheel3_rpm >= 0 ? true : false, std::clamp(abs(wheel3_rpm), 0, 999));
    _sendPwm(0x03, NULL, wheel4_rpm >= 0 ? true : false, std::clamp(abs(wheel4_rpm), 0, 999));
}

void MecanunmControl::_topic_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    double xrpm = msg->linear.x / (2 * M_PI * MECANUNM_DIA) * 60;
    double yrpm = msg->linear.y / (2 * M_PI * MECANUNM_DIA) * 60;
    double yaw = msg->angular.z;
    //RCLCPP_INFO(this->get_logger(), "xrpm: %lf, yrpm: %lf, yaw: %lf\n", xrpm, yrpm, yaw);
    _moveChassis(xrpm, yrpm, yaw);
}

MecanunmControl::MecanunmControl(const rclcpp::NodeOptions &options)
    : MecanunmControl("", options) {}

MecanunmControl::MecanunmControl(const std::string &name_space,
                                 const rclcpp::NodeOptions &options)
    : Node("mecanum_control", name_space, options) {
    _publisher = this->create_publisher<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", rclcpp::QoS(10));
    _subscription = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10),
        std::bind(&MecanunmControl::_topic_callback, this,
                  std::placeholders::_1));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanunmControl>());
    rclcpp::shutdown();
    return 0;
}
