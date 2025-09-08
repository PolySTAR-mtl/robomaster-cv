/** \file monitor_video.cpp
 * \brief Video monitor node executable
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// ROS includes

#include "rclcpp/rclcpp.hpp"

// Local includes

#include "video_monitor.hpp"

/** \brief This node shows the current video stream as well as the detected
 * bounding boxes, tracklets and current target
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto monitor = std::make_shared<VideoMonitor>("monitor_video");

    rclcpp::spin(monitor);
    rclcpp::shutdown();
    return 0;
}
