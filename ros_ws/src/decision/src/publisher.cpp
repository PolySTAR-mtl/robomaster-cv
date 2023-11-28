#include "ros/ros.h"
#include "tracking/Tracklets.h"

#include <sstream>
#include <string_view>

tracking::Tracklet createTracklet(std::string_view id, float x, float y, float w, 
                    float h, std::uint8_t clss, float score){
    tracking::Tracklet trk;
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
  ros::init(argc, argv, "tracking");

  ros::NodeHandle nh("~");

  ros::Publisher chatter_pub = nh.advertise<tracking::Tracklets>("tracklets", 1000);

  ros::Rate loop_rate(1);

  int count = 0;
  int i = 0;
  while (ros::ok())
  {

    auto contained1 = createTracklet("ArmureStd", 25, 25, 10, 10, static_cast<std::uint8_t>(1), 0); 
    auto contained2 = createTracklet("ArmureHero1", 525, 525, 1, 1, static_cast<std::uint8_t>(1), 0);  
    auto contained3 = createTracklet("ArmureHero2", 525, 525, 10, 10, static_cast<std::uint8_t>(1), 0);
    auto container1 = createTracklet("Std", 0, 0, 100, 100, static_cast<std::uint8_t>(4), 0);
    auto container2 = createTracklet("Hero", 500, 500, 100, 100, static_cast<std::uint8_t>(5), 0);

    tracking::Tracklets trks;
    trks.tracklets.push_back(contained1);
    trks.tracklets.push_back(contained2);
    trks.tracklets.push_back(contained3);  
    trks.tracklets.push_back(container1);
    trks.tracklets.push_back(container2);

    chatter_pub.publish(trks);

    ros::spinOnce();

    loop_rate.sleep();
    i += 10;
    ++count;
  }
  return 0;
}
