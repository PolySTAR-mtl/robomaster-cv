/** \file odom.cpp
 * \brief Odometry translator class definition
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "Eigen/Core"

#include "nav_msgs/msg/odometry.hpp"

class Odom : public rclcpp::Node {
  public:
    Odom();

    /** \fn handlePos
     * \brief Handle a position message by derivating speed, applying the
     * cinematic model and integrating
     */
    void handlePos(int64_t enc1, int64_t enc2, int64_t enc3, int64_t enc4);

    /** \fn handleSpeed
     * \brief Apply the cinematic model
     */
    void handleSpeed(float v1, float v2, float v3, float v4);

    /** \fn cinematic
     * \brief 4-wheeled mecanum drive cinematic model
     */
    Eigen::Vector3d cinematic(Eigen::Vector4d& wheel_speed);

    /** \fn integrate
     * \brief Integrate the speed to obtain the new position
     */
    Eigen::Vector3d integrate(Eigen::Vector3d& robot_speed, double dt);

  private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_pos;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_speed;
  
    nav_msgs::msg::Odometry last_estimation;

    double wheel_radius;
    double length_x, length_y;
    int64_t encoder_resolution;

    Eigen::Vector4d last_enc;
};
