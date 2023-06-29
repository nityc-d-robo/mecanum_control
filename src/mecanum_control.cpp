#include "mecanum_control/mecanum_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <motor_lib/motor_lib.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

constexpr int MECANUNM_DIA = 0.152;
constexpr double ROBOT_CENTER_TO_WHEEL_DISTANCE = 0.3; // ロボットの重心からメカナムホイールまでの距離

void moveChassis(double _xrpm, double _yrpm, double _yaw) {
    /// TODO:制御式落とし込んでsendSpeed()する
    double speed_abs = sqrt(pow(_xrpm, 2) + pow(_yrpm, 2));
    double radwimps = atan2(_yrpm, _xrpm);

    double wheel1_rpm = ((speed_abs * sin(radwimps)) + (speed_abs * cos(radwimps)) + (2*sqrt(2)*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4 * M_PI * MECANUNM_DIA;
    double wheel2_rpm = ((speed_abs * sin(radwimps)) - (speed_abs * cos(radwimps)) - (2*sqrt(2)*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4 * M_PI * MECANUNM_DIA;
    double wheel3_rpm = ((speed_abs * sin(radwimps)) + (speed_abs * cos(radwimps)) - (2*sqrt(2)*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4 * M_PI * MECANUNM_DIA;
    double wheel4_rpm = ((speed_abs * sin(radwimps)) - (speed_abs * cos(radwimps)) + (2*sqrt(2)*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4 * M_PI * MECANUNM_DIA;

    MotorLib::md.sendSpeed(0x00, NULL, wheel1_rpm >= 0 ? true : false, abs(wheel1_rpm), 180, 1000, 5000);
    MotorLib::md.sendSpeed(0x01, NULL, wheel2_rpm >= 0 ? true : false, abs(wheel2_rpm), 180, 1000, 5000);
    MotorLib::md.sendSpeed(0x02, NULL, wheel3_rpm >= 0 ? true : false, abs(wheel3_rpm), 180, 1000, 5000);
    MotorLib::md.sendSpeed(0x03, NULL, wheel4_rpm >= 0 ? true : false, abs(wheel4_rpm), 180, 1000, 5000);
}

void MecanunmControl::_topic_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    double xrpm = msg->linear.x / (2 * M_PI * MECANUNM_DIA) * 60;
    double yrpm = msg->linear.y / (2 * M_PI * MECANUNM_DIA) * 60;
    double yaw = msg->angular.z;
    moveChassis(xrpm, yrpm, yaw);
}

MecanunmControl::MecanunmControl(const rclcpp::NodeOptions &options)
    : MecanunmControl("", options) {}

MecanunmControl::MecanunmControl(const std::string &name_space,
                                 const rclcpp::NodeOptions &options)
    : Node("mecanum_control", name_space, options) {
    _subscription = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10),
        std::bind(&MecanunmControl::_topic_callback, this,
                  std::placeholders::_1));
}

int main(int argc, char *argv[]) {
    MotorLib::usb_config.vendor_id = 0x483;
    MotorLib::usb_config.product_id = 0x5740;
    MotorLib::usb_config.b_interface_number = 0;

    MotorLib::usb.setUsb(&MotorLib::usb_config);
    MotorLib::usb.openUsb();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanunmControl>());
    rclcpp::shutdown();
    return 0;
}