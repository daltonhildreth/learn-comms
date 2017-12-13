#ifndef BOUND_VOLUME_H
#define BOUND_VOLUME_H

#include <vector>
#include <glm/glm.hpp>

class Circ;
class Rect;

//either a cylindrical BV or an axis-aligned rectangular BV.
class BoundVolume {
public:
    enum class volume_type { CIRC, RECT };

    //TODO dalton: remove origin, if possible
    //TODO: remove redundancies that need to be synced.
    glm::vec2 _o;//origin
    //what type this is instantiated as; manual polymorphism
    volume_type _vt;

    virtual bool collides(glm::vec2 p) = 0;
    virtual bool line_of_sight(glm::vec2 a, glm::vec2 b,
        glm::vec2 Lab, float len2) = 0;
    virtual float intersect(glm::vec2 bo, glm::vec2 v) = 0;
    virtual std::vector<BoundVolume*> minkowski_sum(BoundVolume*) = 0;
    virtual ~BoundVolume();
protected:
    BoundVolume();
    BoundVolume(glm::vec2 o, volume_type vt);
};

class Rect : public BoundVolume {
public:
    Rect();
    Rect(glm::vec2 o, float w, float h);
    float _w;//width
    float _h;//height

    bool collides(glm::vec2 p);
    bool line_of_sight(glm::vec2 a, glm::vec2 b, glm::vec2 Lab, float len2);
    float intersect(glm::vec2 bo, glm::vec2 v);
    std::vector<BoundVolume*> minkowski_sum(BoundVolume*);
    std::vector<BoundVolume*> minkowski_sum_(Rect* b);
    std::vector<BoundVolume*> minkowski_sum_(Circ* b);
private:
    // broken : but it asks if the lines cross
    //deprecated
    bool line_segs_collide(glm::vec2 p1, glm::vec2 p2, glm::vec2 p3, glm::vec2 p4);
    //axis = 0: the line is x = val {oValLo < y < oValHi} // vertical
    //axis = 1: the line is y = val {oValLo < x < oValHi} // horizontal
    //asks if pp1->pp2 crosses the above axial-line
    //deprecated
    bool line_axial_line_collide(glm::vec2 pp1, glm::vec2 pp2, float val,
        int axis, float oValLo, float oValHi);
};

class Circ : public BoundVolume {
public:
    Circ();
    Circ(glm::vec2 o, float r);
    //radius
    float _r;

    bool collides(glm::vec2 p);
    bool line_of_sight(glm::vec2 a, glm::vec2 b, glm::vec2 Lab, float len2);
    float intersect(glm::vec2 bo, glm::vec2 v);
    std::vector<BoundVolume*> minkowski_sum(BoundVolume*);
    std::vector<BoundVolume*> minkowski_sum_(Rect* b);
    std::vector<BoundVolume*> minkowski_sum_(Circ* b);
};

#endif//BOUND_VOLUME_H
