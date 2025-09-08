/** \file monitor_logger.cpp
 * \brief Logger node executable
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// ROS includes

#include "rclcpp/rclcpp.hpp"

/** \brief This node prints every relevant informations and serves as a watchdog
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("monitor_logger");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
