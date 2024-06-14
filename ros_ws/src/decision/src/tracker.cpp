/** \file tracker.cpp
 * \brief abstract class for trackers
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 * \author Thomas Petrie <thomas.petrie@polymtl.ca>
 */

#include "tracker.h"

Tracker::Tracker(ros::NodeHandle& n, int _enemy_color) : nh(n), enemy_color(_enemy_color), tListener(tBuffer) {
        sub_tracklets = nh.subscribe("tracklets", 1,
                                     &Tracker::callbackTracklets, this);

        pub_target = nh.advertise<serial::Target>("target", 1);
        std::cout << "Enemy color set to be: "
                  << (enemy_color == 0 ? "red" : "blue") << "\n";
}

serial::Target Tracker::toTarget(tracking::Tracklet& trk) {
        serial::Target target;

        std::cout << "Det : " << trk.x << " ( " << trk.w << " ) " << trk.y
                  << " ( " << trk.h << " )\n";

        auto x_c = static_cast<float>(trk.x + trk.w / 2.f - center_x);
        auto y_c = static_cast<float>(trk.y + trk.h / 2.f - center_y);

        std::cout << "x_c = " << x_c << " ; y_c = " << y_c << '\n';

        // Simple approximation .. if we consider x_c & y_c to be low enough
        int16_t theta = std::floor(y_c * alpha_y * 1000.f);
        int16_t phi = std::floor(x_c * alpha_x * 1000.f);

        tf2::Quaternion qTurret;

        try {
            auto transformTurret =
                tBuffer.lookupTransform("base_link", "turret", ros::Time(0));
            tf2::convert(transformTurret.transform.rotation, qTurret);
        } catch (tf2::LookupException e) {
            // Couldn't find lookup. Keep identity
            qTurret = tf2::Quaternion::getIdentity();
        }

        double roll, pitch, yaw;
        tf2::Matrix3x3 m(qTurret);
        m.getRPY(roll, pitch, yaw);

        std::cout << "Turret : p = " << pitch << ", y = " << yaw << '\n';

        target.theta = pitch - theta;
        target.phi = yaw + phi;
        target.dist = 2000u; // 2 m
        target.located = true;
        target.stamp = ros::Time::now();
        target.distance_center = std::hypot(x_c, y_c);

        return target;
    }