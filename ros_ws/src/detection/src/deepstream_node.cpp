/** \file deepstream_node.cpp
 * \brief Main detection node for the deepstream-app, executable
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// Local includes

#include "deepstream_detector.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto detector = std::make_shared<DeepstreamDetector>();

    rclcpp::spin(detector);
    rclcpp::shutdown();

    return 0;
}
