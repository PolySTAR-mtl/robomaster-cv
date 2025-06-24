/** \file simple_tracker.cpp
 * \brief Simple targeting node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// Std includes

#include <algorithm>
#include <memory>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "polystar_msgs/msg/target.hpp"
#include "polystar_msgs/msg/tracklets.hpp"


class SimpleTracker : public rclcpp::Node {
  public:
    SimpleTracker()
        : Node("simple_tracker"), tBuffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tListener(*tBuffer) {
        // Set initial values from parameters
        enemy_color = get_parameter("enemy_color").as_int();
        center_x = get_parameter("trim_x").as_double();
        center_y = get_parameter("trim_y").as_double();

        // Create parameter callback
        param_callback_handle = this->add_on_set_parameters_callback(
            std::bind(&SimpleTracker::parametersCallback, this, std::placeholders::_1));

        sub_tracklets = this->create_subscription<polystar_msgs::msg::Tracklets>(
            "tracklets", 1,
            std::bind(&SimpleTracker::callbackTracklets, this, std::placeholders::_1));

        pub_target = this->create_publisher<polystar_msgs::msg::Target>("target", 1);
        
        RCLCPP_INFO(this->get_logger(), "Enemy color set to be: %s",
                    (enemy_color == 0 ? "red" : "blue"));
    }

    void callbackTracklets(const polystar_msgs::msg::Tracklets::SharedPtr trks) {
        auto distance = [](auto d1, auto d2) {
            return std::sqrt(std::pow(d1.x - d2.x, 2) +
                             std::pow(d1.y - d2.y, 2));
        };
        float best_dist = INFINITY;
        int index = -1;

        int i = 0;

        for (auto trk : trks->tracklets) {
            auto dist = distance(last_trk, trk);
            if (dist < best_dist && enemy_color == int(trk.clss)) {
                index = i;
                best_dist = dist;
            }
            ++i;
        }

        if (index != -1) {
            last_trk = trks->tracklets[index];
            pub_target->publish(toTarget(last_trk));
        }
    }

    polystar_msgs::msg::Target toTarget(const polystar_msgs::msg::Tracklet& trk) {
        polystar_msgs::msg::Target target;

        RCLCPP_DEBUG(this->get_logger(), "Det : %f ( %f ) %f ( %f )", 
                     trk.x, trk.w, trk.y, trk.h);

        auto x_c = static_cast<float>(trk.x + trk.w / 2.f - center_x);
        auto y_c = static_cast<float>(trk.y + trk.h / 2.f - center_y);

        RCLCPP_DEBUG(this->get_logger(), "x_c = %f ; y_c = %f", x_c, y_c);

        // Simple approximation .. if we consider x_c & y_c to be low enough
        int16_t theta = std::floor(y_c * alpha_y * 1000.f);
        int16_t phi = std::floor(x_c * alpha_x * 1000.f);

        tf2::Quaternion qTurret;

        try {
            auto transformTurret =
                tBuffer->lookupTransform("base_link", "turret", tf2::TimePointZero);
            tf2::convert(transformTurret.transform.rotation, qTurret);
        } catch (tf2::LookupException& e) {
            // Couldn't find lookup. Keep identity
            qTurret = tf2::Quaternion::getIdentity();
        }

        RCLCPP_DEBUG(this->get_logger(), "Quaternion: %f %f %f %f", 
                     qTurret.x(), qTurret.y(), qTurret.z(), qTurret.w());

        double roll, pitch, yaw;
        tf2::Matrix3x3 m(qTurret);
        m.getRPY(roll, pitch, yaw);

        RCLCPP_DEBUG(this->get_logger(), "Turret : p = %f, y = %f", pitch, yaw);

        target.theta = pitch - theta;
        target.phi = yaw + phi;
        target.dist = 2000u; // 2 m
        target.located = true;
        target.stamp = this->now();
        target.distance_center = std::hypot(x_c, y_c);

        return target;
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto& param : parameters) {
            if (param.get_name() == "enemy_color") {
                enemy_color = param.as_int();
                RCLCPP_INFO(this->get_logger(), "New enemy color set to %s",
                            (enemy_color == 0 ? "red" : "blue"));
            } else if (param.get_name() == "trim_x") {
                center_x = param.as_double();
                RCLCPP_INFO(this->get_logger(), "New trim_x: %f", center_x);
            } else if (param.get_name() == "trim_y") {
                center_y = param.as_double();
                RCLCPP_INFO(this->get_logger(), "New trim_y: %f", center_y);
            }
        }

        return result;
    }

  private:
    rclcpp::Subscription<polystar_msgs::msg::Tracklets>::SharedPtr sub_tracklets;
    rclcpp::Publisher<polystar_msgs::msg::Target>::SharedPtr pub_target;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
    int enemy_color;

    polystar_msgs::msg::Tracklet last_trk;

    float center_x = 416.f / 2.f;
    float center_y = 416.f / 2.f;

    // Scaling factor
    float alpha_y = 0.0007;
    float alpha_x = 0.0014;

    std::shared_ptr<tf2_ros::Buffer> tBuffer;
    tf2_ros::TransformListener tListener;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleTracker>(); // Default to red enemy
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
