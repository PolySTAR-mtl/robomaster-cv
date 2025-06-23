/** \file detector.hpp
 * \brief Detection node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#ifndef _POLYSTAR_DEEPSTREAM_DETECTOR_HPP
#define _POLYSTAR_DEEPSTREAM_DETECTOR_HPP

// ROS includes


#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/Image.h>

#include "polystar_msgs/msg/detections.h"

extern "C" {
void deepstreamCallback(void*, void*);
}

/** \class Detector
 */
class DeepstreamDetector : public rclcpp::Node {
  public:
    /** ctor
     * \brief Main constructor. Loads the weights
     */
    DeepstreamDetector();

    /** \fn run
     * \brief Launch the gstreamer pipeline
     */
    void run();

    /** \fn callback
     * \brief Function to call to publish detections
     */
    void callback(polystar_msgs::msg::Detections&);

  private:
    void setupNet(const std::string& deepstream_config);

    rclcpp::Publisher<polystar_msgs::msg::Detections>::SharedPtr pub_detections;

    int fake_argc;
    std::vector<const char*> fake_argv;
};

#endif
