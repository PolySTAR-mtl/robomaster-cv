/** \file video_monitor.hpp
 * \brief Video monitor class
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#ifndef _POLYSTAR_VIDEO_MONITOR_H
#define _POLYSTAR_VIDEO_MONITOR_H

// Std includes

#include <random>
#include <unordered_map>

// Ros includes

#include <opencv2/core/core.hpp>

#include "rclcpp/rclcpp.hpp"

#include "polystar_msgs/msg/detections.hpp"
#include "sensor_msgs/msg/image.hpp"

class VideoMonitor : public rclcpp::Node {
  public:
    VideoMonitor(const std::string& default_class = "Error class");

    // ----- ROS Callbacks ----- //
    /** \fn callbackImage
     * \brief Callback for images from the camera
     */
    void callbackImage(const std::shared_ptr<const sensor_msgs::msg::Image>& im);

    /** \fn callbackDetections
     * \brief Callback for detections coming from the detection node (NN)
     */
    void callbackDetections(const std::shared_ptr<const polystar_msgs::msg::Detections>& dets);

  private:
    /** \fn getColor
     * \brief Returns the color corresponding to a class, or generates on
     *
     * \details Class "colors" are generated randomly and then stored in a map
     * to ensure that the same color is consistently attributed to the same
     * class.
     */
    cv::Scalar getColor(uint8_t cls);

    /** \fn getClassName
     * \brief Returns the name associated to a class number
     *
     * \details Class names are loaded from the parameter server, and cached in
     * a vector
     */
    const std::string& getClassName(uint8_t cls);

    // ROS
    rclcpp::Subscription<polystar_msgs::msg::Detections>::SharedPtr sub_detections;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_im;

    // Internals
    cv::Mat curr_image;
    std::unordered_map<uint8_t, cv::Scalar> colormap;
    std::vector<std::string> classmap;
    std::random_device randgen;

    const std::string default_name;
};

#endif
