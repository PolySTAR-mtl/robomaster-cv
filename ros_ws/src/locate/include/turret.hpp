/** \file turret.hpp
 * \brief Turret position definition
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "polystar_msgs/msg/turret_feedback.hpp"

class Turret : public rclcpp::Node {
  public:
    Turret() : Node("turret"), pub_pos(this) {
      this->get_parameter("robot.turret.heigh", turret_height);
    } 

    void callbackTurret(const std::shared_ptr<const polystar_msgs::msg::TurretFeedback>& turret);

  private:
    tf2_ros::TransformBroadcaster pub_pos;

    float turret_height;
};
