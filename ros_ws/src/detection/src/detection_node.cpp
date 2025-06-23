/** \file detection_node.cpp
 * \brief Main detection node, executable
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// Local includes

#include "detector.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("detection");

    std::string datacfg, config_path, weights_path;
    
    if (!node->get_parameter("net.datacfg", datacfg)) {
        throw std::runtime_error("Network datacfg path not specified");
    }

    if (!node->get_parameter("net.config_path", config_path)) {
        throw std::runtime_error("Network config path not specified");
    }

    if (!node->get_parameter("net.weights", weights_path)) {
        throw std::runtime_error("Network weights path not specified");
    }

    auto detector = std::make_shared<Detector>(datacfg, config_path, weights_path);

    rclcpp::spin(detector);
    rclcpp::shutdown();

    return 0;
}
