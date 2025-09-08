/** \file detection_node.cpp
 * \brief Main detection node, executable
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// Local includes

#include "detector.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);


    auto detector = std::make_shared<Detector>();

    rclcpp::spin(detector);
    rclcpp::shutdown();

    return 0;
}
