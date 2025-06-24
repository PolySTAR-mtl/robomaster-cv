#include "rclcpp/rclcpp.hpp"
#include "polystar_msgs/msg/tracklets.hpp"

#include <sstream>
#include <string_view>

polystar_msgs::msg::Tracklet createTracklet(std::string_view id, float x, float y, float w, 
                    float h, std::uint8_t clss, float score){
    polystar_msgs::msg::Tracklet trk;
    trk.id = id;
    trk.x = x;
    trk.y = y;
    trk.w = w;
    trk.h = h;
    trk.clss = clss;
    trk.score = score;

    return trk;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("tracking_publisher");
  
  auto publisher = node->create_publisher<polystar_msgs::msg::Tracklets>("tracklets", 10);

  rclcpp::Rate loop_rate(1);

  int count = 0;
  int i = 0;
  while (rclcpp::ok())
  {
    auto contained1 = createTracklet("ArmureStd", 25, 25, 10, 10, static_cast<std::uint8_t>(1), 0); 
    auto contained2 = createTracklet("ArmureHero1", 925, 925, 1, 1, static_cast<std::uint8_t>(1), 0);  
    auto contained3 = createTracklet("ArmureHero2", 925, 925, 10, 10, static_cast<std::uint8_t>(1), 0);
    auto soloModule = createTracklet("ArmureSolo", 175, 175, 50, 50, 1, 0);
    auto container1 = createTracklet("Std", 0, 0, 100, 100, static_cast<std::uint8_t>(4), 0);
    auto container2 = createTracklet("Hero", 900, 900, 100, 100, static_cast<std::uint8_t>(5), 0);

    auto message = std::make_unique<polystar_msgs::msg::Tracklets>();
    message->tracklets.push_back(contained1);
    message->tracklets.push_back(contained2);
    message->tracklets.push_back(contained3);
    message->tracklets.push_back(soloModule);  
    message->tracklets.push_back(container1);
    message->tracklets.push_back(container2);

    publisher->publish(std::move(message));

    rclcpp::spin_some(node);
    loop_rate.sleep();
    i += 10;
    ++count;
  }
  return 0;
}
