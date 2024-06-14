#pragma once
#include "tracking/Tracklets.h"


enum class RoboType : int { Base = 3, Standard = 4, Hero = 5, Sentry = 6 };

class BoundingBox {
    friend class WeightedTracker;
    
    const float upper_edge;
    const float lower_edge;
    const float left_edge;
    const float right_edge;
    float width;
    float height;
    float x;
    float y;
    float score;

    std::string id;
    int clss;

    static float weightBase;
    static float weightStandard;
    static float weightHero;
    static float weightSentry;
    static float weightSize;
    static float weightDist;

    static float constexpr scaleFactor = 1.2f;

    BoundingBox(float x = 0.f, float y = 0.f, float upper_edge = 0.f, float lower_edge = 0.f, 
                float left_edge = 0.f, float right_edge = 0.f, std::uint8_t clss = 0, std::string id = "Basic", float score = 0);
    BoundingBox(tracking::Tracklet& bbox);

    float getSize();
    float getDistance(BoundingBox other);
    float roboType(int enemy_color, const tracking::TrackletsConstPtr& trks);
    float scoreToReturn(int enemy_color, float scoreToReturn, const tracking::TrackletsConstPtr& trks);
    bool contains(BoundingBox* inner);
};