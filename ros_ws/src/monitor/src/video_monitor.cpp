/** \file video_monitor.cpp
 * \brief Video monitor class
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// Local includes

#include "video_monitor.hpp"

// Ros includes

#include <cv_bridge/cv_bridge.hpp>

// ----- Consts ----- //
// TODO : Move to parameters

constexpr int MONITOR_FONT_SIZE = 5;
constexpr auto MONITOR_FONT_FACE = cv::FONT_HERSHEY_SIMPLEX;
const static cv::Scalar MONITOR_FONT_COLOR(255., 255., 255.);
constexpr int MONITOR_FONT_PADDING = 5;

// ----- Methods ----- //

VideoMonitor::VideoMonitor(const std::string& class_name)
    : Node("video_monitor"), default_name(class_name) {
    pub_im = create_publisher<sensor_msgs::msg::Image>("image_out", 1); 

    sub_cam = create_subscription<sensor_msgs::msg::Image>("image_in", 1, 
        [this](sensor_msgs::msg::Image::ConstSharedPtr im) { callbackImage(im); });
    sub_detections = create_subscription<polystar_msgs::msg::Detections>("detections", 1, 
        [this](polystar_msgs::msg::Detections::ConstSharedPtr dets) { callbackDetections(dets); });

    classmap = this->declare_parameter<std::vector<std::string>>("classmap", std::vector<std::string>{});

    if (classmap.empty()) {
        RCLCPP_INFO(
            this->get_logger(),
            "No classmap found, using default name %s",
            default_name.c_str()
        );
    }
}

void VideoMonitor::callbackImage(const std::shared_ptr<const sensor_msgs::msg::Image>& im) {
    auto img_bridged = cv_bridge::toCvShare(im);

    // Saving the last image to draw on top when receiving detections
    curr_image = img_bridged->image.clone();
}

cv::Scalar VideoMonitor::getColor(uint8_t cls) {
    auto search = colormap.find(cls);
    if (search != std::end(colormap)) {
        return search->second;
    } else {
        cv::Scalar new_color(randgen() % 255, randgen() % 255, randgen() % 255);
        colormap.insert({cls, new_color});
        return new_color;
    }
}

const std::string& VideoMonitor::getClassName(uint8_t cls) {
    if (cls >= classmap.size()) {
        return default_name;
    } else {
        return classmap[cls];
    }
}

void VideoMonitor::callbackDetections(
    const std::shared_ptr<const polystar_msgs::msg::Detections>& dets) {
    if (curr_image.empty()) {
        // No image saved yet
        return;
    }

    auto img_rects = curr_image.clone();
    for (auto& det : dets->detections) {
        auto color = getColor(det.clss);
        cv::Point p1{static_cast<int>(det.x - det.w / 2),
                     static_cast<int>(det.y - det.h / 2)};
        cv::Point p2{static_cast<int>(det.x + det.w / 2),
                     static_cast<int>(det.y + det.h / 2)};

        cv::rectangle(img_rects, p1, p2, color);

        // Print text
        p2 = {p2.x, p1.y};
        p1 = {p1.x, p1.y - 2 * MONITOR_FONT_PADDING - MONITOR_FONT_SIZE};
        cv::rectangle(img_rects, p1, p2, color, cv::FILLED);
        p1.y += MONITOR_FONT_PADDING + MONITOR_FONT_SIZE;

        cv::putText(img_rects, getClassName(det.clss), p1, MONITOR_FONT_FACE,
                    static_cast<double>(MONITOR_FONT_SIZE) / 12.,
                    cv::Scalar(MONITOR_FONT_COLOR));
    }

    std::shared_ptr<sensor_msgs::msg::Image> out_msg =
        cv_bridge::CvImage(dets->header, sensor_msgs::image_encodings::BGR8,
                           img_rects)
            .toImageMsg();
    pub_im->publish(*out_msg);
}
