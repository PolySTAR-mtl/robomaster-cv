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

    // Test 1 -> Taille vs Distance du milieu
    auto solo1 = createTracklet("PetitProche", 200, 200, 10, 10, static_cast<uint8_t>(1), 0);
    auto solo2 = createTracklet("MoyenProche", 180, 180, 60, 60, static_cast<uint8_t>(1), 0);
    auto solo3 = createTracklet("GrandProche", 150, 150, 90, 90, static_cast<uint8_t>(1), 0);
    auto solo4 = createTracklet("PetitLoin", 50, 50, 10, 10, static_cast<uint8_t>(1), 0);
    auto solo5 = createTracklet("MoyenLoin", 40, 40, 60, 60, static_cast<uint8_t>(1), 0);
    auto solo6 = createTracklet("GrandLoin", 25, 25, 90, 90, static_cast<uint8_t>(1), 0);
    auto solo7 = createTracklet("PetitMid", 145, 145, 25, 25, static_cast<uint8_t>(1), 0);
    auto solo8 = createTracklet("MoyenMid", 128, 128, 60, 60, static_cast<uint8_t>(1), 0);
    auto solo9 = createTracklet("GrandMid", 113, 113, 90, 90, static_cast<uint8_t>(1), 0);
    

    tracking::Tracklets trks;
    trks.tracklets.push_back(solo1);
    trks.tracklets.push_back(solo2);
    trks.tracklets.push_back(solo3);
    trks.tracklets.push_back(solo4);
    trks.tracklets.push_back(solo5);
    trks.tracklets.push_back(solo6);
    trks.tracklets.push_back(solo7);
    trks.tracklets.push_back(solo8);
    trks.tracklets.push_back(solo9);

    chatter_pub.publish(trks);

    ros::spinOnce();

    loop_rate.sleep();
    i += 10;
    ++count;
  }
  return 0;
}
