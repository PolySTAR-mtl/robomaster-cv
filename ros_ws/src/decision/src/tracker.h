#pragma once
#include "rclcpp/rclcpp.hpp"
#include "polystar_msgs/msg/target.hpp"
#include "polystar_msgs/msg/tracklets.hpp"

class Tracker : public rclcpp::Node {
  public: 
    Tracker();

    virtual polystar_msgs::msg::Target toTarget(polystar_msgs::msg::Tracklet& trk) = 0;
    virtual void callbackTracklets(const polystar_msgs::msg::Tracklets::SharedPtr trks) = 0;


    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);

  protected:    
    rclcpp::Subscription<polystar_msgs::msg::Tracklets>::SharedPtr sub_tracklets;
    rclcpp::Publisher<polystar_msgs::msg::Target>::SharedPtr pub_target;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

    int enemy_color;
    
    polystar_msgs::msg::Tracklet last_trk;

    int im_w = 416/2;
    int im_h = 416/2;
    
    // Scaling factor
    float alpha_y = 0.001;
    float alpha_x = 0.01;
};
