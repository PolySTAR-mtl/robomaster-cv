#pragma once

// ROS includes

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "serial/Target.h"
#include "tracking/Tracklets.h"
#include "bounding_box.h"

class Tracker {
    public: 
    Tracker(ros::NodeHandle& n, int _enemy_color);

    virtual serial::Target toTarget(tracking::Tracklet& trk);
    virtual void callbackTracklets(const tracking::TrackletsConstPtr& trks) = 0;

    protected:    
    ros::Subscriber sub_tracklets;
    ros::Publisher pub_target;
    ros::NodeHandle& nh;

    tf2_ros::Buffer tBuffer;
    tf2_ros::TransformListener tListener;

    int enemy_color;
    
    tracking::Tracklet last_trk;

    float center_x = 416.f / 2.f;
    float center_y = 416.f / 2.f;

    int im_w = 416/2;
    int im_h = 416/2;
    
    // Scaling factor
    float alpha_y = 0.0007;
    float alpha_x = 0.0014;
};