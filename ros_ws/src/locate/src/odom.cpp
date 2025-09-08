/** \file odom.cpp
 * \brief Odometry translator class implementation
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#include "odom.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

std::array<double, 36> unknown_covariance;

double toRadiants(int64_t enc, int64_t resolution) {
    return static_cast<double>(enc) * 2 * M_PI /
           static_cast<double>(resolution);
}

Odom::Odom() : Node("odom") {
    pub_pos = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    pub_speed = this->create_publisher<nav_msgs::msg::Odometry>("odom_speed", 1);

    this->get_parameter("robot.wheel_radius", wheel_radius);
    this->get_parameter("robot.l_x", length_x);
    this->get_parameter("robot.l_y", length_y);
    this->get_parameter("robot.encoder_resolution", encoder_resolution);
    
    unknown_covariance.fill(-1.);
}

void Odom::handlePos(int64_t enc1, int64_t enc2, int64_t enc3, int64_t enc4) {
    auto rad = [this](int64_t enc) {
        return toRadiants(enc, encoder_resolution);
    };

    Eigen::Vector4d enc(rad(enc1), rad(enc2), rad(enc3), rad(enc4));

    auto robot_pose = cinematic(enc);

    // Generate Odom message
    nav_msgs::msg::Odometry odom;

    odom.header.frame_id = "odom";
    odom.header.stamp = this->now();

    odom.child_frame_id = "base_link";

    // Pose
    odom.pose.covariance = unknown_covariance;
    odom.pose.pose.position.x = robot_pose[0];
    odom.pose.pose.position.y = robot_pose[1];
    odom.pose.pose.position.z = 0.;

    tf2::Quaternion rot;
    rot.setRPY(0., 0., robot_pose[2]);

    odom.pose.pose.orientation.w = rot.w();
    odom.pose.pose.orientation.x = rot.x();
    odom.pose.pose.orientation.y = rot.y();
    odom.pose.pose.orientation.z = rot.z();

    odom.twist.covariance = unknown_covariance;

    pub_pos->publish(odom);

    last_enc = enc;
}

Eigen::Vector3d Odom::cinematic(Eigen::Vector4d& vel) {
    auto v_x = (vel[0] + vel[1] + vel[2] + vel[3]) * wheel_radius / 4.;

    auto v_y =
        (-1. * vel[0] + vel[1] + vel[2] + -1. * vel[3]) * wheel_radius / 4.;

    auto omega = (-1. * vel[0] + vel[1] + -1 * vel[2] + vel[3]) * wheel_radius /
                 (4. * (length_x + length_y));

    return {v_x, v_y, omega};
}

Eigen::Vector3d Odom::integrate(Eigen::Vector3d& speed, double dt) {
    auto x = last_estimation.pose.pose.position.x + speed[0] * dt;
    auto y = last_estimation.pose.pose.position.y + speed[1] * dt;

    auto& last_q = last_estimation.pose.pose.orientation;
    tf2::Quaternion q(last_q.x, last_q.y, last_q.z, last_q.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    yaw += speed[2] * dt;

    return {x, y, yaw};
}

void Odom::handleSpeed(float v1, float v2, float v3, float v4) {
    Eigen::Vector4d speed(v1, v2, v3, v4);

    auto robot_speed = cinematic(speed);

    // Generate Odom message
    nav_msgs::msg::Odometry odom;

    odom.header.frame_id = "odom_speed";
    odom.header.stamp = this->now();

    odom.child_frame_id = "odom_speed";

    /*
    // Pose
    odom.pose.covariance = unknown_covariance;
    odom.pose.pose.position.x = robot_pose[0];
    odom.pose.pose.position.y = robot_pose[1];
    odom.pose.pose.position.z = 0.;

    tf2::Quaternion rot;
    rot.setRPY(0., 0., robot_pose[2]);

    odom.pose.pose.orientation.w = rot.w();
    odom.pose.pose.orientation.x = rot.x();
    odom.pose.pose.orientation.y = rot.y();
    odom.pose.pose.orientation.z = rot.z();

    odom.twist.covariance = unknown_covariance;

    pub_pos.publish(odom);

    last_enc = enc;
    */
}
