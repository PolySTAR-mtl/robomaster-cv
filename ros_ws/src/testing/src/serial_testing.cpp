/** \file serial_testing.cpp
 * \brief Serial interface integration test
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// Std includes

#include <cmath>

// ROS includes

#include <rclcpp/rclcpp.hpp>

// Local includes

#include "polystar_msgs/msg/target.hpp"

constexpr uint16_t rewrap_pi_millirad(uint16_t angle) {
    uint16_t circle_millirad = 2 * M_PI * 1000;
    if (angle > circle_millirad) {
        return angle % circle_millirad;
    } else {
        return angle;
    }
}

/** \brief This node sends dummy orders to the serial port at a specified
 * frequency
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("serial_testing");

    constexpr float amplitude = M_PI / 3.f;
    constexpr uint16_t dist = 100u;
    float freq, increment;

    node->get_parameter("freq", freq);
    node->get_parameter("increment", increment);

    auto pub = node->create_publisher<polystar_msgs::msg::Target>("target", 1);

    rclcpp::Rate rate(freq);

    while (rclcpp::ok()) {
        for (float alpha = 0.f; alpha < 2 * M_PI; alpha += increment) {
            polystar_msgs::msg::Target msg;

            int16_t theta = std::floor(std::sin(alpha) * amplitude * 1000.f);
            int16_t phi = std::floor(std::cos(alpha) * amplitude * 1000.f);

            msg.located = true;
            msg.theta = theta;
            msg.phi = phi;
            msg.dist = dist;

            pub->publish(msg);

            rate.sleep();
            rclcpp::spin_some(node);
        }
    }
}
