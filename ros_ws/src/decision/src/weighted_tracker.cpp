/** \file weighted_tracker.cpp
 * \brief Simple targeting node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 * \author Thomas Petrie <thomas.petrie@polymtl.ca>
 */

// Std includes

#include <algorithm>

// OpenCV Includes

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

// Project includes

#include "tracker.h"

int16_t radToMillirad(float rad) { return static_cast<int16_t>(rad * 1000); }

class WeightedTracker : Tracker {

  public:
    WeightedTracker(ros::NodeHandle& n, int _enemy_color)
        : Tracker(n, _enemy_color){


        // Init weights
        BoundingBox::weightBase = nh.param("weights/base", 20.f);
        BoundingBox::weightStandard = nh.param("weights/std", 40.f);
        BoundingBox::weightHero = nh.param("weights/hro", 100.f);
        BoundingBox::weightSentry = nh.param("weights/sty", 30.f);
        BoundingBox::weightSize = nh.param("weights/size", 0.01);
        BoundingBox::weightDist = nh.param("weights/dist", 1.f);
    }

    void callbackTracklets(const tracking::TrackletsConstPtr& trks) override {
        BoundingBox basic(center_x, center_y);
        BoundingBox* best_target = &basic;

        std::cout << "\nBasic. \n" << "id: " << best_target->id << 
        " x: "<< best_target->x << " y: "<< best_target->y << " w: "<< best_target->width << " h: "<< 
        best_target->height << " class: "<< static_cast<int>(best_target->clss) << " score: "<< best_target->score << "\n";

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
        tracking::Tracklet target;
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
        pub_target.publish(toTarget(target));
        
    };
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
