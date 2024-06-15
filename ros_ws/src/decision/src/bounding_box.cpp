/** \file bounding_box.cpp
 * \brief defines all boundingbox object operations
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 * \author Thomas Petrie <thomas.petrie@polymtl.ca>
 */

#include "bounding_box.h"

 //Default Constructor 
BoundingBox::BoundingBox(float x, float y, float upper_edge, float lower_edge,
    float left_edge, float right_edge, std::uint8_t clss, const std::string id, float score) 
    : x(x), y(y), upper_edge(upper_edge), lower_edge(lower_edge), left_edge(left_edge), 
      right_edge(right_edge), clss(clss), id(id), score(score){
        width = right_edge - left_edge;
        height = upper_edge - lower_edge;
      } 

// Construction from tracklet
BoundingBox::BoundingBox(tracking::Tracklet& bbox) 
    : x(bbox.x), y(bbox.y), upper_edge(bbox.y),
      lower_edge(bbox.y + bbox.h), left_edge(bbox.x),
      right_edge(bbox.x + bbox.w), clss(bbox.clss), width(bbox.w),
      height(bbox.h), id(bbox.id) {}

float BoundingBox::getSize() { return this->width * this->height; }

float BoundingBox::getDistance(BoundingBox other) { 
    return std::sqrt(std::pow(this->x + (this->right_edge - this->left_edge)/2 - other.x, 2) + std::pow(this->y + (this->lower_edge - this->upper_edge)/2 - other.y, 2));
    }

// If the bounding box is not a container but an armor module, its type score will be 0
// If it is a container, we enter the scoreToReturn function with the correct score type
float BoundingBox::roboType(int enemy_color, const tracking::TrackletsConstPtr& trks) {
    switch (this->clss) {  
    case static_cast<int>(RoboType::Base):
        return scoreToReturn(enemy_color, weightBase, trks); 
    case static_cast<int>(RoboType::Standard):
        return scoreToReturn(enemy_color, weightStandard, trks);
    case static_cast<int>(RoboType::Hero):
        return scoreToReturn(enemy_color, weightHero, trks);
    case static_cast<int>(RoboType::Sentry):
        return scoreToReturn(enemy_color, weightSentry, trks);
    default:
        return 0.f;
    }
}

// If we found an enemy armor module, we return the correct type score.
float BoundingBox::scoreToReturn(int enemy_color, float scoreToReturn, const tracking::TrackletsConstPtr& trks) {
    bool found = false;

    for (auto trk : trks->tracklets) {
        BoundingBox inner(trk);
        if (this->contains(&inner) && inner.clss == enemy_color) {        
            found = true;
        }
    }

    if(found)                    
        return scoreToReturn;   
    return 0;                   
}

// We check to see if a BoundingBox is contained within another (scaled up by a scale factor)
bool BoundingBox::contains(BoundingBox* inner) { // Checks to see if all bounds of inner bounding box are within this one's bounds   

    if(this->id == inner->id) return false;
    // Creating a bigger, scaled, version of this bounding box
    float centerX = this->x + (this->width / 2);
    int centerY = this->y + (this->height / 2);

    int newH = this->height * scaleFactor;
    int newW = this->width * scaleFactor;

    int newLowerEdge = centerY + newH /2;
    int newUpperEdge = centerY - newH /2; 
    int newLeftEdge = centerX - newW /2;
    int newRightEdge = centerX + newW /2; 

    // Checking if inner box is within new scaled bounds
    bool isInside = ((newUpperEdge < inner->y) && (newLowerEdge > inner->y) &&
        (newLeftEdge < inner->x) && (newRightEdge > inner->x));

    return isInside;
}
