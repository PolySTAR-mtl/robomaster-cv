#pragma once
#include <ros/ros.h>
#include "serial/Target.h"
#include "tracking/Tracklets.h"

class Tracker {
    public: 
    Tracker(ros::NodeHandle& n, int _enemy_color);

    virtual serial::Target toTarget(tracking::Tracklet& trk) = 0;
    virtual void callbackTracklets(const tracking::TrackletsConstPtr& trks) = 0;

    protected:    
    ros::Subscriber sub_tracklets;
    ros::Publisher pub_target;
    ros::NodeHandle& nh;

    int enemy_color;
    
    tracking::Tracklet last_trk;

    int im_w = 416/2;
    int im_h = 416/2;
    
    // Scaling factor
    float alpha_y = 0.001;
    float alpha_x = 0.01;
};