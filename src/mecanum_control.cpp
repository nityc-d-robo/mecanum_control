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

void moveChassis(double _xrpm, double _yrpm, double _yaw) {
    /// TODO:制御式落とし込んでsendSpeed()する
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