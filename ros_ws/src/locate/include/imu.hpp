/** \file imu.cpp
 * \brief IMU translator class definition
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"

class IMU : public rclcpp::Node {
  public:
    IMU() : Node("imu") {
        pub_msg = create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    }

    void handle(float ax, float ay, float az, float rx, float ry, float rz);

  private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_msg;

    constexpr static const auto frame_id = "imu";
};
