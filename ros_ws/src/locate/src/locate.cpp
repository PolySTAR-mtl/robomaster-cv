/** \file locate.cpp
 * \brief Locate node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#include "imu.hpp"
#include "odom.hpp"
#include "turret.hpp"

#include "rclcpp/rclcpp.hpp"

#include "polystar_msgs/msg/position_feedback.hpp"

void handleMessage(const std::shared_ptr<const polystar_msgs::msg::PositionFeedback>& pos, IMU& imu,
                   Odom& odom) {
    imu.handle(pos->imu_ax, pos->imu_ay, pos->imu_az, pos->imu_rx, pos->imu_ry,
               pos->imu_rz);
    odom.handlePos(pos->enc_1, pos->enc_2, pos->enc_3, pos->enc_4);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("locate");

    IMU imu;
    Odom odom;
    Turret turret;

    auto sub_pos = node->create_subscription<polystar_msgs::msg::PositionFeedback>(
        "polystar_msgs.position", 1, [&imu, &odom](const std::shared_ptr<const polystar_msgs::msg::PositionFeedback>& pos) -> void {
            handleMessage(pos, imu, odom);
        });

    auto sub_turret = node->create_subscription<polystar_msgs::msg::TurretFeedback>(
        "polystar_msgs.turret", 1, [&turret](const std::shared_ptr<const polystar_msgs::msg::TurretFeedback>& msg) {
            turret.callbackTurret(msg);
        });

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
    }

    return 0;
}
