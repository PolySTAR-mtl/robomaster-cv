/** \file serial_interface.cpp
 * \brief Serial interface node executable
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// ROS includes

#include "rclcpp/rclcpp.hpp"

// Local includes

#include "serial_spinner.hpp"

/** \brief This node serves as the main interface to the serial port
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SerialSpinner>());

    rclcpp::shutdown();

    return 0;
}
