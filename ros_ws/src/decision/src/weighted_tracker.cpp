/** \file weighted_tracker.cpp
 * \brief Simple targeting node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 * \author Thomas Petrie <thomas.petrie@polymtl.ca>
 */

// Std includes

#include <algorithm>

// ROS includes

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

// OpenCV Includes

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

// Project includes

#include "tracker.h"
#include "polystar_msgs/msg/target.h"
#include "bounding_box.h"

int16_t radToMillirad(float rad) { return static_cast<int16_t>(rad * 1000); }

class WeightedTracker : public Tracker {

  public:
    WeightedTracker()
        : Tracker(), tBuffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tListener(*tBuffer) {

            
            // Declare default values        
            declare_parameter<double>("weights/base", 20.0);
            declare_parameter<double>("weights/std", 40.0);
            declare_parameter<double>("weights/hro", 100.0);
            declare_parameter<double>("weights/sty", 30.0);
            declare_parameter<double>("weights/size", 0.01);
            declare_parameter<double>("weights/dist", 1.0);

            declare_parameter<double>("focal_length", 3.04e-3f);
            declare_parameter<double>("pixel_size", 1.2e-6f);
            declare_parameter<std::vector<double>>("camera/camera_matrix/data", std::vector<double>{});
            declare_parameter<std::vector<double>>("camera/distortion_coefficients/data", std::vector<double>{});
            declare_parameter<int>("camera/image_width", 640);
            declare_parameter<int>("camera/image_height", 480);


            // Init weights
            BoundingBox::weightBase = get_parameter("weights/base").as_double();
            BoundingBox::weightStandard = get_parameter("weights/std").as_double();
            BoundingBox::weightHero = get_parameter("weights/hro").as_double();
            BoundingBox::weightSentry = get_parameter("weights/sty").as_double();
            BoundingBox::weightSize = get_parameter("weights/size").as_double();
            BoundingBox::weightDist = get_parameter("weights/dist").as_double();

            // Init camera matrix and distortion coefficients
            
            camera_matrix = get_parameter("camera/camera_matrix/data").as_double_array();
            distorsion_coeffs = get_parameter("camera/distortion_coefficients/data").as_double_array();
            im_w = get_parameter("camera/image_width").as_int();
            im_h = get_parameter("camera/image_height").as_int();

            focal_length = get_parameter("focal_length").as_double();
            pixel_size = get_parameter("pixel_size").as_double();

            initMap();
        }

    void callbackTracklets(const polystar_msgs::msg::Tracklets::SharedPtr trks) override {
        BoundingBox basic;
        BoundingBox* best_target = &basic;

        std::vector<BoundingBox> boxes;

        // Assign individual scores to all bounding boxes
        for (auto trk : trks->tracklets){
            BoundingBox tracklet(trk);

            tracklet.score = 0;

            std::cout << "\nReceived Tracklet: \n" << "id: " << trk.id << 
            " x: "<< trk.x << " y: "<< trk.y << " w: "<< trk.w << " h: "<< 
            trk.h << " class: "<< static_cast<int>(trk.clss) << " score: "<< trk.score << "\n";

            std::cout << "Corresponding Bbox: \n" << "upper_edge: "<< tracklet.upper_edge << " lower_edge: "
            << tracklet.lower_edge << " left_edge: "<< tracklet.left_edge << " right_edge: "<< tracklet.right_edge << "\n";

            // The roboType function also assigns parents and children boxes
            // A type score is 0 if the tracklet is an armor module or doesn't contain enemy armor modules 
            float type = tracklet.roboType(enemy_color, trks);

            tracklet.score += type; 

            std::cout << "\n";

            if(tracklet.clss == enemy_color){
                float size = tracklet.getSize();          
                float dist = tracklet.getDistance(*best_target); 
                    
                tracklet.score += size * BoundingBox::weightSize;
                tracklet.score += 1000/(dist * BoundingBox::weightDist);
            }

            boxes.push_back(tracklet);

        }

        std::cout << "Armor module scores: \n";

        // Add outer score to armor modules
        for(BoundingBox box : boxes){

            std::cout << box.id << ": " << box.score << "\n";

            if(box.clss == static_cast<int>(RoboType::Base) || box.clss == static_cast<int>(RoboType::Standard) || 
                box.clss == static_cast<int>(RoboType::Hero) || box.clss == static_cast<int>(RoboType::Sentry)){
                
                for(int i = 0; i < boxes.size(); i++){
                    if(box.contains(&(boxes.at(i))) && boxes.at(i).clss == enemy_color){
                        std::cout << box.id << " contains " << boxes.at(i).id << "\n";
                        boxes.at(i).score += box.score;
                    }
                }
            }
        }

        std::cout << "\nUpdated scores: \n";

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
        polystar_msgs::msg::Tracklet target;
        target.id = best_target->id;
        target.x = best_target->x;
        target.y = best_target->y;
        target.w = best_target->width;
        target.h = best_target->height;
        target.clss = best_target->clss;
        target.score = best_target->score;

        std::cout << "\nPublished Tracklet. \n" << "id: " << target.id << 
        " x: "<< target.x << " y: "<< target.y << " w: "<< target.w << " h: "<< 
        target.h << " class: "<< static_cast<int>(target.clss) << " score: "<< target.score << "\n";
        pub_target->publish(toTarget(target));
        
    };

    polystar_msgs::msg::Target toTarget(polystar_msgs::msg::Tracklet& trk) override {
        polystar_msgs::msg::Target target;

        std::cout << "Det : " << trk.x << " ( " << trk.w << " ) " << trk.y
                  << " ( " << trk.h << " )\n";

        cv::Mat pixel_image({trk.x, trk.y});
        cv::Mat pixel_undistort(2, 1, CV_32FC1);

        pixel_undistort.at<float>(0) = mat1.at<float>(trk.y, trk.x);
        pixel_undistort.at<float>(1) = mat2.at<float>(trk.y, trk.x);

        cv::Mat x(cv::Point3f{pixel_undistort.at<float>(0),
                              pixel_undistort.at<float>(1), 1.f});

        cv::Mat y;
        cv::solve(new_c, x, y);

        std::cout << "solve\n" << y << '\n';

        y.at<float>(0) /= y.at<float>(2);
        y.at<float>(1) /= y.at<float>(2);

        tf2::Quaternion qTarget, qTurret;
        qTarget.setRPY(0., std::atan(y.at<float>(1)),
                       std::atan(y.at<float>(0)));

        auto transformTurret =
            tBuffer->lookupTransform("base_link", "turret", rclcpp::Time(0));
        tf2::convert(transformTurret.transform.rotation, qTurret);

        //qTarget *= qTurret;
        qTurret *= qTarget;

        tf2::Matrix3x3 m(qTurret);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        int16_t theta = radToMillirad(pitch);
        int16_t phi = radToMillirad(yaw);

        std::cout << "    Trk : \n"
                  << pixel_image << "\n    Undistord\n"
                  << pixel_undistort << "\n    y\n"
                  << y << '\n';

        target.theta = theta;
        target.phi = phi;
        target.dist = 2000u; // 2 m
        target.located = true;
        target.stamp = now();

        return target;
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
    std::shared_ptr<tf2_ros::Buffer> tBuffer;
    tf2_ros::TransformListener tListener;

    std::vector<double> camera_matrix;
    std::vector<double> distorsion_coeffs;

    cv::Mat new_c, mat1, mat2, im_center;

    float focal_length;
    float pixel_size;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto tracker = std::make_shared<WeightedTracker>();
    
    rclcpp::spin(tracker);
    rclcpp::shutdown();
}

float BoundingBox::weightBase;
float BoundingBox::weightStandard;
float BoundingBox::weightHero;
float BoundingBox::weightSentry;
float BoundingBox::weightSize;
float BoundingBox::weightDist;
