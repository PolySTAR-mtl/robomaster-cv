/** \file turret.hpp
 * \brief Turret position definition
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

#include "ros/ros.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/QuaternionStamped.h"
#include "polystar_msgs/msg/TurretFeedback.h"

class Turret {
  public:
    Turret(ros::NodeHandle& n)
        : nh(n), turret_height(nh.param("/robot/turret/height", 0.5)) {}

    void callbackTurret(const polystar_msgs::TurretFeedbackPtr& turret);

  private:
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster pub_pos;

    float turret_height;
};
