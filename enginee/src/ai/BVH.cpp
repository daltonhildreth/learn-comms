#include "BVH.h"
#include "Pool.h"

BVH::BVH() : o(nullptr), right(nullptr) {
}

//strongly assumes that every Entity has a BV.
BVH::BVH(std::vector<Entity*> objects) {
    size_ = objects.size();
    if (objects.size() == 0) {
        o = nullptr;
        right = nullptr;
        return;
    }

    std::vector<Index> sorted_x(objects.size());
    std::vector<Index> sorted_z(objects.size());
    for (size_t i = 0; i < objects.size(); i++) {
        //for sorted_x Index is (idx of obj for x, idx of zidx)
        //for sorted_z Index is (idx of obj for z, idx of xidx)
        //so .obj will always grab the object and .oth the other table's index
        //to the same object
        sorted_x[i] = sorted_z[i] = Index(static_cast<unsigned>(i), 0);
    }

    //sort by dimension
    std::sort(sorted_x.begin(), sorted_x.end(), [&](Index a, Index b) {
        auto bva = *POOL.get<BoundVolume*>(*objects[a.obj]);
        auto bvb = *POOL.get<BoundVolume*>(*objects[b.obj]);
        return bva->_o.x < bvb->_o.x;
    });
    //connect index tables
    for (size_t i = 0; i < sorted_x.size(); i++) {
        sorted_z[sorted_x[i].obj].oth = static_cast<unsigned>(i);
    }

    std::sort(sorted_z.begin(), sorted_z.end(), [&](Index a, Index b) {
        auto bva = *POOL.get<BoundVolume*>(*objects[a.obj]);
        auto bvb = *POOL.get<BoundVolume*>(*objects[b.obj]);
        return bva->_o.y < bvb->_o.y;
    });
    //connect index tables
    for (size_t i = 0; i < sorted_z.size(); i++) {
        sorted_x[sorted_z[i].oth].oth = static_cast<unsigned>(i);
    }

    construct_(objects, sorted_x, sorted_z);
}

BVH::~BVH() {
    if (!is_leaf()) {
        delete left;
        delete right;
    }
}

std::vector<Entity*> BVH::query(BoundVolume* q) {
    std::vector<Entity*> NN;
    if (left == nullptr) {
        return std::vector<Entity*>();
    }

    if (q->_vt == BoundVolume::volume_type::CIRC) {
        query_(static_cast<Circ*>(q), NN);
    } else {
        query_(dynamic_cast<Rect*>(q), NN);
    }
    return NN;
}

void BVH::query_(Rect* q, std::vector<Entity*>& NN) {
    if (is_leaf()) {
        if (o != nullptr) {
            auto& bv = **(POOL.get<BoundVolume*>(*o));
            if (bv._vt == BoundVolume::volume_type::CIRC) {
                if (circ_rect_collider_(static_cast<Circ*>(&bv), q)) {
                    NN.push_back(o);
                }
            }
            else {
                if (rect_rect_collider_(q, static_cast<Rect*>(&bv))) {
                    NN.push_back(o);
                }
            }
        }
    }
    else {
        if (rect_rect_collider_(q, &aabb)) {
            left->query_(q, NN);
            right->query_(q, NN);
        }
    }
}

void BVH::query_(Circ* q, std::vector<Entity*>& NN) {
    if (is_leaf()) {
        if (o != nullptr) {
            auto& bv = **(POOL.get<BoundVolume*>(*o));
            if (bv._vt == BoundVolume::volume_type::CIRC) {
                if (circ_circ_collider_(q, static_cast<Circ*>(&bv))) {
                    NN.push_back(o);
                }
            }
            else {
                if (circ_rect_collider_(q, static_cast<Rect*>(&bv))) {
                    NN.push_back(o);
                }
            }
        }
    }
    else {
        if (circ_rect_collider_(q, &aabb)) {
            left->query_(q, NN);
            right->query_(q, NN);
        }
    }
}

bool BVH::rect_rect_collider_(Rect* q, Rect* r) {
    float w = q->_w + r->_w;
    float h = q->_h + r->_h;
    return Rect(r->_o, w, h).collides(q->_o);
}

bool BVH::circ_rect_collider_(Circ* q, Rect* r) {
    glm::vec2 L = r->_o - q->_o;
    L /= sqrt(glm::dot(L, L));
    L *= q->_r;
    L += q->_o;
    return q->collides(r->_o) || r->collides(L);
}

bool BVH::circ_circ_collider_(Circ* q, Circ* c) {
    glm::vec2 diff = q->_o - c->_o;
    float r = q->_r + c->_r;
    return glm::dot(diff, diff) < r*r;
}

bool BVH::is_leaf() {
    return right == nullptr;
}

size_t BVH::size() {
    return size_;
}

void BVH::construct_(
        std::vector<Entity*> objects,
        std::vector<Index> sorted_x,
        std::vector<Index> sorted_z) {
    assert(sorted_x.size() == sorted_z.size());
    assert(sorted_x.size() >= 1);
    size_ = sorted_x.size();
    if (sorted_x.size() == static_cast<size_t>(1)) {
        assert(sorted_x[0].obj == sorted_z[0].obj);
        right = nullptr;
        o = objects[sorted_x[0].obj];
        return;
    }

    //create bounding volume for this level, remembering the min/max in each dim
    float min_x, max_x, min_z, max_z;
    min_x = min_z = std::numeric_limits<float>::max();
    max_x = max_z = -std::numeric_limits<float>::max();
    //tightly fit min_x/max_x/min_z/max_z
    for (size_t i = 0; i < sorted_x.size(); i++) {
        auto& bv = **POOL.get<BoundVolume*>(*objects[sorted_x[i].obj]);
        float dim_x, dim_z;
        if (bv._vt == BoundVolume::volume_type::RECT) {
            //add a nudge if you want fat BVs
            dim_x = static_cast<Rect*>(&bv)->_w/2;
            dim_z = static_cast<Rect*>(&bv)->_h/2;
        }
        else {
            dim_x = dim_z = static_cast<Circ*>(&bv)->_r;
        }

        float x_lo = bv._o.x - dim_x;
        float x_hi = bv._o.x + dim_x;
        float z_lo = bv._o.y - dim_z;
        float z_hi = bv._o.y + dim_z;
        if (x_lo < min_x) min_x = x_lo;
        if (x_hi > max_x) max_x = x_hi;
        if (z_lo < min_z) min_z = z_lo;
        if (z_hi > max_z) max_z = z_hi;
    }

    float dx = max_x - min_x;
    float dz = max_z - min_z;
    aabb = Rect(glm::vec2(min_x + dx/2, min_z + dz/2), dx, dz);

    //partition along longer axis, splitting indices equally
    std::vector<Index> x_rhs, x_lhs;
    std::vector<Index> z_rhs, z_lhs;
    if (dx > dz) {
        split_(sorted_x, sorted_z, x_lhs, z_lhs, x_rhs, z_rhs);
    } else {
        split_(sorted_z, sorted_x, z_lhs, x_lhs, z_rhs, x_rhs);
    }

    //probably the slowest part is the new, tbh XD
    left = new BVH();
    left->construct_(objects, x_lhs, z_lhs);
    right = new BVH();
    right->construct_(objects, x_rhs, z_rhs);
}

void BVH::split_(
        std::vector<Index>& sorted_a,
        std::vector<Index>& sorted_b,
        std::vector<Index>& a_lhs,
        std::vector<Index>& b_lhs,
        std::vector<Index>& a_rhs,
        std::vector<Index>& b_rhs) {
    assert(sorted_a.size() == sorted_b.size());
    unsigned half = static_cast<unsigned>(static_cast<float>(sorted_a.size())/2.f);
    unsigned half_ceil = static_cast<unsigned>(static_cast<float>(sorted_a.size())/2.f + .5f);
    a_lhs = std::vector<Index>(half);
    b_lhs = std::vector<Index>(half);
    a_rhs = std::vector<Index>(half_ceil);
    b_rhs = std::vector<Index>(half_ceil);

    unsigned umax = std::numeric_limits<unsigned>::max();

    for (size_t i = 0; i < sorted_a.size(); ++i) {
        if (i < half) {
            a_lhs[i] = Index(sorted_a[i].obj, umax);
        } else {
            a_rhs[i - half] = Index(sorted_a[i].obj, umax);
        }
    }
    for (unsigned i = 0, lhs = 0, rhs = 0; i < static_cast<unsigned>(sorted_b.size()); ++i) {
        unsigned oth_i = sorted_b[i].oth;
        if (oth_i < half) {
            b_lhs[lhs] = Index(sorted_b[i].obj, oth_i);
            assert(oth_i < a_lhs.size());
            a_lhs[oth_i] = Index(a_lhs[oth_i].obj, lhs);
            ++lhs;
        }
        else {
            b_rhs[rhs] = Index(sorted_b[i].obj, oth_i - half);
            assert(oth_i - half < a_rhs.size());
            a_rhs[oth_i - half] = Index(a_rhs[oth_i - half].obj, rhs);
            ++rhs;
        }
    }

}
