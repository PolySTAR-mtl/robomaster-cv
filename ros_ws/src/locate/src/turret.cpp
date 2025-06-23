/** \file turret.cpp
 * \brief Turret position implementation
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#include "turret.hpp"

void Turret::callbackTurret(const std::shared_ptr<const polystar_msgs::msg::TurretFeedback>& turret) {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = turret->stamp;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "turret";

    tf.transform.translation.x = 0.;
    tf.transform.translation.y = 0.;
    tf.transform.translation.z = turret_height;

    tf2::Quaternion rot;
    rot.setRPY(0., turret->pitch, turret->yaw);
    tf.transform.rotation.x = rot.x();
    tf.transform.rotation.y = rot.y();
    tf.transform.rotation.z = rot.z();
    tf.transform.rotation.w = rot.w();

    pub_pos.sendTransform(tf);
}
