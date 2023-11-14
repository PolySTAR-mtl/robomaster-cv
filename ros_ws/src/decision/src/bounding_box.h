#pragma once
#include "tracking/Tracklets.h"

class BoundingBox {
    friend class WeightedTracker;
    
    const float upper_edge;
    const float lower_edge;
    const float left_edge;
    const float right_edge;
    float width;
    float height;
    const float x;
    const float y;
    float score;

    BoundingBox* container;
    std::vector<BoundingBox> containedArray;

    const std::string id;
    int clss;

    static float weightBase;
    static float weightStandard;
    static float weightHero;
    static float weightSentry;
    static float weightSize;
    static float weightDist;

    static float constexpr scaleFactor = 1.2f;

    BoundingBox(const float x = 0.f, const float y = 0.f, const float upper_edge = 0.f, const float lower_edge = 0.f, 
                const float left_edge = 0.f, const float right_edge = 0.f, const int clss = 0, const std::string id = "Basic");
    BoundingBox(tracking::Tracklet& bbox);

    int getSize();
    float roboType(int enemy_color, const tracking::TrackletsConstPtr& trks);
    std::vector<BoundingBox> findBoxes(const tracking::TrackletsConstPtr& trks);
    float scoreReturn(int enemy_color, float scoreToReturn, const tracking::TrackletsConstPtr& trks);
    BoundingBox scaled(BoundingBox other);
    bool contains(BoundingBox& inner);
};