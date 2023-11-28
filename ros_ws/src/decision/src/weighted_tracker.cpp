/** \file weighted_tracker.cpp
 * \brief Simple targeting node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 * \author Thomas Petrie <thomas.petrie@polymtl.ca>
 */

// Std includes

#include <algorithm>

// ROS includes

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// OpenCV Includes

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

// Project includes

#include "serial/Target.h"
#include "bounding_box.h"

int16_t radToMillirad(float rad) { return static_cast<int16_t>(rad * 1000); }

class WeightedTracker {

  public:
    WeightedTracker(ros::NodeHandle& n, int _enemy_color)
        : tListener(tBuffer), nh(n), enemy_color(_enemy_color) {
        sub_tracklets = nh.subscribe("tracklets", 1,
                                     &WeightedTracker::callbackTracklets, this);

        pub_target = nh.advertise<serial::Target>("target", 1);

        // Init weights
        BoundingBox::weightBase = nh.param("weights/base", 20.f);
        BoundingBox::weightStandard = nh.param("weights/std", 40.f);
        BoundingBox::weightHero = nh.param("weights/hro", 100.f);
        BoundingBox::weightSentry = nh.param("weights/sty", 30.f);
        BoundingBox::weightSize = nh.param("weights/size", 0.125);
        BoundingBox::weightDist = nh.param("weights/dist", 1.f);

        // Init camera matrix and distortion coefficients
        bool cam_param = true;
        cam_param &= nh.getParam("/camera/camera_matrix/data", camera_matrix);
        cam_param &= nh.getParam("/camera/distortion_coefficients/data",
                                 distorsion_coeffs);
        cam_param &= nh.getParam("/camera/image_width", im_w);
        cam_param &= nh.getParam("/camera/image_height", im_h);

        if (!cam_param) {
            throw std::runtime_error("WeightedTracker::WeightedTracker() : "
                                     "Could not fetch camera parameters");
        }

        focal_length = nh.param("focal_length", 3.04e-3f);
        pixel_size = nh.param("pixel_size", 1.2e-6f);

        initMap();

        std::cout << "Enemy color set to be: "
                  << (enemy_color == 0 ? "red" : "blue") << "\n";
    }

    void callbackTracklets(const tracking::TrackletsConstPtr& trks) {
        BoundingBox basic;
        BoundingBox* best_target = &basic;

        std::vector<BoundingBox> boxes;

        // Assign individual scores to all bounding boxes
        for (auto trk : trks->tracklets){
            BoundingBox tracklet(trk);

            tracklet.score = 0;

            std::cout << "Received Tracklet. \n" << "id: " << trk.id << 
            " x: "<< trk.x << " y: "<< trk.y << " w: "<< trk.w << " h: "<< 
            trk.h << " class: "<< static_cast<int>(trk.clss) << " score: "<< trk.score << "\n";

            // The roboType function also assigns parents and children boxes
            // A type score is 0 if the tracklet is an armor module or doesn't contain enemy armor modules 
            float type = tracklet.roboType(enemy_color, trks);

            tracklet.score += type; 

            if(tracklet.clss == enemy_color){
                float size = tracklet.getSize();          
                float dist = tracklet.getDistance(*best_target); 
                    
                tracklet.score += size * BoundingBox::weightSize;
                tracklet.score += dist * BoundingBox::weightDist;
            }

            boxes.push_back(tracklet);

        }

        // Add outer score to armor modules
        for(BoundingBox box : boxes){

            std::cout << box.id << ": " << box.score << "\n";

            if(box.clss == static_cast<int>(RoboType::Base) || box.clss == static_cast<int>(RoboType::Standard) || 
                box.clss == static_cast<int>(RoboType::Hero) || box.clss == static_cast<int>(RoboType::Sentry)){
                
                for(int i = 0; i < boxes.size(); i++){
                    if(box.contains(&(boxes.at(i))) && boxes.at(i).clss == enemy_color){
                        boxes.at(i).score += box.score;
                    }
                }
            }
        }

        for(BoundingBox box : boxes){
            std::cout << box.id << ": " << box.score << "\n";

            if(box.score > best_target->score){
                best_target->id = box.id;
                best_target->x = box.x;
                best_target->y = box.y;
                best_target->width = box.width;
                best_target->height = box.height;
                best_target->clss = box.clss;
                best_target->score = box.score;
            }
        }
        
        // Publish the best tracklet
        tracking::Tracklet target;
        target.id = best_target->id;
        target.x = best_target->x;
        target.y = best_target->y;
        target.w = best_target->width;
        target.h = best_target->height;
        target.clss = best_target->clss;
        target.score = best_target->score;

        std::cout << "Published Tracklet. \n" << "id: " << target.id << 
        " x: "<< target.x << " y: "<< target.y << " w: "<< target.w << " h: "<< 
        target.h << " class: "<< static_cast<int>(target.clss) << " score: "<< target.score << "\n";
        pub_target.publish(toTarget(target));
        
    };

    serial::Target toTarget(tracking::Tracklet& trk) {
        // simple tracker version 

        serial::Target target;

        std::cout << "Det : " << trk.x << " ( " << trk.w << " ) " << trk.y
                  << " ( " << trk.h << " )\n";

        auto x_c = trk.x + trk.w / 2 - im_w / 2;
        auto y_c = trk.y + trk.h / 2 - im_h / 2;

        std::cout << "x_c = " << x_c << " ; y_c = " << y_c << '\n';

        // Simple approximation .. if we consider x_c & y_c to be low enough
        int16_t theta = std::floor(y_c * alpha_y * 1000.f);
        int16_t phi = std::floor(x_c * alpha_x * 1000.f);

        target.theta = theta;
        target.phi = phi;
        target.dist = 2000u; // 2 m
        target.located = true;
        target.stamp = ros::Time::now();

        return target;
        
        // weighted tracker version
        // serial::Target target;

        // std::cout << "Det : " << trk.x << " ( " << trk.w << " ) " << trk.y
        //           << " ( " << trk.h << " )\n";

        // cv::Mat pixel_image({trk.x, trk.y});
        // cv::Mat pixel_undistort(2, 1, CV_32FC1);

        // pixel_undistort.at<float>(0) = mat1.at<float>(trk.y, trk.x);
        // pixel_undistort.at<float>(1) = mat2.at<float>(trk.y, trk.x);

        // cv::Mat x(cv::Point3f{pixel_undistort.at<float>(0),
        //                       pixel_undistort.at<float>(1), 1.f});

        // cv::Mat y;
        // cv::solve(new_c, x, y);

        // std::cout << "solve\n" << y << '\n';

        // y.at<float>(0) /= y.at<float>(2);
        // y.at<float>(1) /= y.at<float>(2);

        // tf2::Quaternion qTarget, qTurret;
        // qTarget.setRPY(0., std::atan(y.at<float>(1)),
        //                std::atan(y.at<float>(0)));

        // auto transformTurret =
        //     tBuffer.lookupTransform("base_link", "turret", ros::Time(0));
        // tf2::convert(transformTurret.transform.rotation, qTurret);

        // qTarget *= qTurret;

        // tf2::Matrix3x3 m(qTarget);

        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);

        // int16_t theta = radToMillirad(pitch);
        // int16_t phi = radToMillirad(yaw);

        // std::cout << "    Trk : \n"
        //           << pixel_image << "\n    Undistord\n"
        //           << pixel_undistort << "\n    y\n"
        //           << y << '\n';

        // target.theta = theta;
        // target.phi = -phi;
        // target.dist = 2000u; // 2 m
        // target.located = true;
        // target.stamp = ros::Time::now();

        // return target;
    }

    void initMap() {
        cv::Mat c(3, 3, CV_32F, camera_matrix.data());
        cv::Mat d(5, 1, CV_32F, distorsion_coeffs.data());
        cv::Point im_size{im_w, im_h};
        new_c = cv::getOptimalNewCameraMatrix(c, d, im_size, 1.f, cv::Size(), 0,
                                              true);
        cv::initUndistortRectifyMap(c, d, cv::Mat(), new_c, im_size, CV_32F,
                                    mat1, mat2);

        std::cout << "c\n" << c << '\n';
        std::cout << "new_c\n" << new_c << '\n';

        im_center = cv::Mat(2, 1, CV_32F);
        im_center.at<float>(0) = new_c.at<float>(0, 2) / new_c.at<float>(0, 0);
        im_center.at<float>(1) = new_c.at<float>(1, 2) / new_c.at<float>(1, 1);

        std::cout << "im_center" << im_center << '\n';
    }

  private:
    tf2_ros::Buffer tBuffer;
    tf2_ros::TransformListener tListener;

    std::vector<float> camera_matrix;
    std::vector<float> distorsion_coeffs;

    cv::Mat new_c, mat1, mat2, im_center;

    ros::NodeHandle& nh;
    ros::Subscriber sub_tracklets;
    ros::Publisher pub_target;
    int enemy_color;

    tracking::Tracklet last_trk;

    int im_w;
    int im_h;
    
    // Scaling factor
    float alpha_y = 0.001;
    float alpha_x = 0.01;

    float focal_length;
    float pixel_size;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "decision");
    ros::NodeHandle nh("~");

    int enemy_color;

    if (!nh.getParam("enemy_color", enemy_color)) {
        throw std::runtime_error("Enemy color not specified");
    }
    if (enemy_color != 0 and enemy_color != 1) {
        throw std::runtime_error("Enemy color should be 0 (red) or 1 (blue)");
    }

    WeightedTracker tracker(nh, enemy_color);

    ros::spin();
}

float BoundingBox::weightBase;
float BoundingBox::weightStandard;
float BoundingBox::weightHero;
float BoundingBox::weightSentry;
float BoundingBox::weightSize;
float BoundingBox::weightDist;
