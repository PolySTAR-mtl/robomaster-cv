/** \file imu.cpp
 * \brief IMU translator class implementation
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#include "imu.hpp"

void IMU::handle(float ax, float ay, float az, float rx, float ry, float rz) {
    sensor_msgs::msg::Imu imu;

    imu.header.frame_id = frame_id;
    imu.header.stamp = this->now();

    // No orientation from our IMU
    // clang-format off
    imu.orientation_covariance = {-1., 0., 0., 
                                  0., -1., 0., 
                                  0., 0., -1.};
    // clang-format on

    imu.angular_velocity.x = rx;
    imu.angular_velocity.y = ry;
    imu.angular_velocity.z = rz;
    // clang-format off
    imu.angular_velocity_covariance = {0., 0., 0., 
                                       0., 0., 0., 
                                       0., 0., 0.};
    // clang-format on

    imu.linear_acceleration.x = ax;
    imu.linear_acceleration.y = ay;
    imu.linear_acceleration.z = az;
    // clang-format off
    imu.linear_acceleration_covariance = {0., 0., 0., 
                                          0., 0., 0., 
                                          0., 0., 0.};
    // clang-format on

    pub_msg->publish(imu);
}
