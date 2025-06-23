/** \file detector.hpp
 * \brief Detection node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#ifndef _POLYSTAR_DETECTOR_HPP
#define _POLYSTAR_DETECTOR_HPP

// ROS includes

#include "rclcpp/rclcpp.hpp"
#include "polystar_msgs/msg/detections.hpp"
#include "sensor_msgs/msg/image.hpp"

/** \class Detector
 */
class Detector : public rclcpp::Node {
  public:
    /**
     * \brief Main constructor. Loads the weights
     */
    Detector();

    /** \brief Destructor
     */
    ~Detector();

    /** \fn imageCallback
     * \brief Callback for incoming images (from camera)
     */
    void imageCallback(const std::shared_ptr<const sensor_msgs::msg::Image>& img);

  private:
    void setupNet(const std::string& datacfg, const std::string& config_path,
                  const std::string& weights_path);

    void loadLabels();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Publisher<polystar_msgs::msg::Detections>::SharedPtr pub_detections_;

    /** PIml idiom
     * \brief Not a fan of PImpl, but in this case it prevents Darknet from
     * leaking a horrendous amount of symbols in the default namespace ...
     * (Thanks, C)
     */
    struct impl;
    std::unique_ptr<impl> p;
};

#endif
