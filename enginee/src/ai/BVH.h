#ifndef BVH_H
#define BVH_H

#include "BoundVolume.h"
#include "Entity.h"
#include <vector>
#include <algorithm>

//BVH now holds any Entity which has a BoundVolume
class BVH {
public:
    BVH();
    BVH(std::vector<Entity*>);
    ~BVH();

    //only use Circ queries please...
    std::vector<Entity*> query(BoundVolume* q);
private:
    void query_(Rect* q, std::vector<Entity*>& NN);
    void query_(Circ* q, std::vector<Entity*>& NN);
    //don't use, test later as the sqrt will probably make this slower
    //this would be useful if it is faster, though, since circ/rect is common
    //that would also mean I need to update my BV code.
    bool circ_rect_collider_(Circ* q, Rect* r);

    bool rect_rect_collider_(Rect* q, Rect* r);

    bool circ_circ_collider_(Circ* q, Circ* c);
public:
    bool is_leaf();
    size_t size();

    union {
        BVH* left;
        Entity* o;
    };
    BVH* right;
    Rect aabb;

private:
    size_t size_;

    struct Index {
        unsigned obj;//object/this
        unsigned oth;//other
        Index() : obj(0), oth(0) {};
        Index(unsigned obj_, unsigned oth_) : obj(obj_), oth(oth_) {};
    };

    void construct_(
        std::vector<Entity*> objects,
        std::vector<Index> sorted_x,
        std::vector<Index> sorted_Z);
    void split_(
            std::vector<Index>& sorted_a,
            std::vector<Index>& sorted_b,
            std::vector<Index>& a_lhs,
            std::vector<Index>& b_lhs,
            std::vector<Index>& a_rhs,
            std::vector<Index>& b_rhs);
};

//http://www.randygaul.net/2013/08/06/dynamic-aabb-tree/
//to avoid excessive use of the heap and indirection
//also, he mentions the use of an AVL tree to maintain balance
/*
struct BVH_node {
    static const int Null = -1;
    bool is_leaf() {
        return right == Null;
    }
    Rect aabb;
    union {
        int parent;
        int next;
    };
    union {
        struct {
            int left;
            int right;
        };
        Object * o;
    };
    int height;
};
*/
#endif//BVH_H
