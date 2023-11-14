#pragma once
#include "tracking/Tracklets.h"

class BoundingBox {
    friend class WeightedTracker;

    const float upper_edge;
    const float lower_edge;
    const float left_edge;
    const float right_edge;
    const float width;
    const float height;
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

    BoundingBox();
    BoundingBox(tracking::Tracklet& bbox);

    int getSize();
    float roboType(int enemy_color, const tracking::TrackletsConstPtr& trks);
    std::vector<BoundingBox> findBoxes(const tracking::TrackletsConstPtr& trks);

    float scoreReturn(int enemy_color, float scoreToReturn, const tracking::TrackletsConstPtr& trks);
    bool contains(BoundingBox& inner);
};