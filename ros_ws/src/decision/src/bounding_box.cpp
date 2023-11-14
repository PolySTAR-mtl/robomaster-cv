#include "bounding_box.h"

enum class RoboType : int { Base = 3, Standard = 4, Hero = 5, Sentry = 6 };

 //Default Constructor 
BoundingBox::BoundingBox(float x, float y, float upper_edge, float lower_edge,
    float left_edge, float right_edge, int clss, const std::string id) 
    : x(x), y(y), upper_edge(upper_edge), lower_edge(lower_edge), left_edge(left_edge), 
      right_edge(right_edge), clss(clss), id(id){
        width = right_edge - left_edge;
        height = upper_edge - lower_edge;
      } 

// Construction from tracklet
BoundingBox::BoundingBox(tracking::Tracklet& bbox) 
    : x(bbox.x), y(bbox.y), upper_edge(bbox.y + bbox.h / 2.f),
      lower_edge(bbox.y - bbox.h / 2.f), left_edge(bbox.x - bbox.w / 2.f),
      right_edge(bbox.x + bbox.w / 2.f), clss(bbox.clss), width(bbox.w),
      height(bbox.h), id(bbox.id) {}

int BoundingBox::getSize() { return this->width * this->height; }

// If the bounding box is not a container but an armor module, its type score will be 0
// If it is a container, we enter the scoreReturn function with the correct score type
float BoundingBox::roboType(int enemy_color, const tracking::TrackletsConstPtr& trks) {
    switch (this->clss) {  
    case static_cast<int>(RoboType::Base):
        return scoreReturn(enemy_color, weightBase, trks); 
    case static_cast<int>(RoboType::Standard):
        return scoreReturn(enemy_color, weightStandard, trks);
    case static_cast<int>(RoboType::Hero):
        return scoreReturn(enemy_color, weightHero, trks);
    case static_cast<int>(RoboType::Sentry):
        return scoreReturn(enemy_color, weightSentry, trks);
    default:
        return 0;
    }
}

// We select all bounding boxes that are within this one
std::vector<BoundingBox> BoundingBox::findBoxes(const tracking::TrackletsConstPtr& trks) {
    std::vector<BoundingBox> contained_boxes;
    for (auto trk : trks->tracklets) { 
        BoundingBox outer(trk);
        if (this->contains(outer)) {
            contained_boxes.push_back(outer); 
        }
    }
    return contained_boxes;
}

// We assign every enemy armor module within this bounding box as this box's children
// If we found any, we return the correct type score
float BoundingBox::scoreReturn(int enemy_color, float scoreToReturn, const tracking::TrackletsConstPtr& trks) {

    std::vector<BoundingBox> contained_boxes = this->findBoxes(trks); 

    bool found = false;

    for (auto& box : contained_boxes) {
        if (box.clss == enemy_color) {       
            box.container = this;            
            this->containedArray.push_back(box);  
            found = true;
        }
    }

    if(found)                    
        return scoreToReturn;   
    return 0;                   
}

// We check to see if a BoundingBox is contained within another (scaled up by a scale factor)
bool BoundingBox::contains(BoundingBox& inner) { // Checks to see if all bounds of a bounding box are within another one's bounds
    BoundingBox temp = BoundingBox(this-> x, this-> y, this->upper_edge * scaleFactor, 
                                   this->lower_edge / scaleFactor, this->left_edge / scaleFactor,
                                   this->right_edge * scaleFactor);
    return ((temp.upper_edge > inner.y) && (temp.lower_edge < inner.y) &&
        (temp.left_edge < inner.x) && (temp.right_edge > inner.x));
}