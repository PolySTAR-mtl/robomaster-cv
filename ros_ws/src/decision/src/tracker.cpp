/** \file tracker.cpp
 * \brief abstract class for trackers
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 * \author Thomas Petrie <thomas.petrie@polymtl.ca>
 */

#include "tracker.h"

Tracker::Tracker() : Node("tracker") {
    sub_tracklets = create_subscription<polystar_msgs::msg::Tracklets>("tracklets", 1,
        [this](const polystar_msgs::msg::Tracklets::SharedPtr m) { callbackTracklets(m); });

        declare_parameter<int>("enemy_color", 0);
        
        enemy_color = get_parameter("enemy_color").as_int();

        pub_target = create_publisher<polystar_msgs::msg::Target>("target", 1);

        // Create parameter callback
        param_callback_handle = add_on_set_parameters_callback(
                std::bind(&Tracker::parametersCallback, this, std::placeholders::_1));

        std::cout << "Enemy color set to be: "
                << (enemy_color == 0 ? "red" : "blue") << "\n";
    }


rcl_interfaces::msg::SetParametersResult Tracker::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto& param : parameters) {
        if (param.get_name() == "enemy_color") {
            enemy_color = param.as_int();
            RCLCPP_INFO(this->get_logger(), "New enemy color set to %s",
                        (enemy_color == 0 ? "red" : "blue"));
        }
    }

    return result;
}
