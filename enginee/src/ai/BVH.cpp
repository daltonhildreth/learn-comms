#include "BVH.h"
#include "Pool.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

BVH::BVH(): o(nullptr), right(nullptr) {}

// strongly assumes that every Entity has a BV.
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
        // for sorted_x Index is (idx of obj for x, idx of zidx)
        // for sorted_z Index is (idx of obj for z, idx of xidx)
        // so .obj will always grab the object and .oth the other table's index
        // to the same object
        sorted_x[i] = sorted_z[i] = Index(static_cast<unsigned>(i), 0);
    }

    // sort by dimension
    std::sort(sorted_x.begin(), sorted_x.end(), [&](Index a, Index b) {
        auto bva = *POOL.get<BoundVolume*>(*objects[a.obj]);
        auto bvb = *POOL.get<BoundVolume*>(*objects[b.obj]);
        return bva->_o.x < bvb->_o.x;
    });
    // connect index tables
    for (size_t i = 0; i < sorted_x.size(); i++) {
        sorted_z[sorted_x[i].obj].oth = static_cast<unsigned>(i);
    }

    std::sort(sorted_z.begin(), sorted_z.end(), [&](Index a, Index b) {
        auto bva = *POOL.get<BoundVolume*>(*objects[a.obj]);
        auto bvb = *POOL.get<BoundVolume*>(*objects[b.obj]);
        return bva->_o.y < bvb->_o.y;
    });
    // connect index tables
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

std::vector<std::pair<Entity*, glm::vec2>> BVH::query(BoundVolume* q) {
    std::vector<std::pair<Entity*, glm::vec2>> NN;
    if (left == nullptr) {
        return std::vector<std::pair<Entity*, glm::vec2>>();
    }

    if (q->_vt == BoundVolume::volume_type::CIRC) {
        query_(static_cast<Circ*>(q), NN);
    } else {
        query_(static_cast<Rect*>(q), NN);
    }
    return NN;
}

void BVH::query_(Rect* q, std::vector<std::pair<Entity*, glm::vec2>>& NN) {
    if (is_leaf()) {
        if (o != nullptr) {
            auto bv = *(POOL.get<BoundVolume*>(*o));
            if (bv == q)
                return;

            if (bv->_vt == BoundVolume::volume_type::CIRC) {
                Circ* bv_circ = static_cast<Circ*>(bv);
                if (circ_rect_collider_(bv_circ, q)) {
                    glm::vec2 on_q = closest_aabb_point_(bv->_o, q);
                    glm::vec2 on_bv = closest_circ_point_(on_q, bv_circ);
                    // normal should push queried object out of collision
                    NN.push_back(std::make_pair(o, on_bv - on_q));
                }
            } else {
                Rect* bv_rect = static_cast<Rect*>(bv);

                if (rect_rect_collider_(q, bv_rect)) {
                    glm::vec2 on_q = closest_aabb_point_(bv->_o, q);
                    glm::vec2 on_bv = closest_aabb_point_(on_q, bv_rect);
                    // normal should push queried object out of collision
                    NN.push_back(std::make_pair(o, on_bv - on_q));
                }
            }
        }
    } else {
        if (rect_rect_collider_(q, &aabb)) {
            left->query_(q, NN);
            right->query_(q, NN);
        }
    }
}

void BVH::query_(Circ* q, std::vector<std::pair<Entity*, glm::vec2>>& NN) {
    if (is_leaf()) {
        if (o != nullptr) {
            auto bv = *(POOL.get<BoundVolume*>(*o));
            if (bv == q)
                return;

            if (bv->_vt == BoundVolume::volume_type::CIRC) {
                Circ* bv_circ = static_cast<Circ*>(bv);
                if (circ_circ_collider_(q, bv_circ)) {
                    glm::vec2 on_q = closest_circ_point_(bv->_o, q);
                    glm::vec2 on_bv = closest_circ_point_(on_q, bv_circ);
                    // normal should push queried object out of collision
                    NN.push_back(std::make_pair(o, on_bv - on_q));
                }
            } else {
                Rect* bv_rect = static_cast<Rect*>(bv);
                if (circ_rect_collider_(q, bv_rect)) {
                    // cross your fingers this isn't too expensive during LMP
                    glm::vec2 on_q = closest_circ_point_(bv->_o, q);
                    glm::vec2 on_bv = closest_aabb_point_(on_q, bv_rect);
                    // normal should push queried object out of collision
                    NN.push_back(std::make_pair(o, on_bv - on_q));
                }
            }
        }
    } else {
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
    glm::vec2 on_q = closest_circ_point_(r->_o, q);
    return q->collides(r->_o) || r->collides(on_q);
    // glm::vec2 on_r = closest_aabb_point_(q->_o, r);
    // return r->collides(q->_o) || q->collides(on_r);
}

bool BVH::circ_circ_collider_(Circ* q, Circ* c) {
    float r = q->_r + c->_r;
    glm::vec2 diff = q->_o - c->_o;
    return glm::length2(diff) < r * r;
}

glm::vec2 BVH::closest_circ_point_(glm::vec2 o_, Circ* c) {
    return c->_r * glm::normalize(o_ - c->_o) + c->_o;
}

glm::vec2 BVH::closest_aabb_point_(glm::vec2 o_, Rect* r) {
    // clamp in all axes to aabb dimensions
    glm::vec2 closest = o_;
    closest.x = glm::clamp(closest.x, r->_o.x - r->_w / 2, r->_o.x + r->_w / 2);
    closest.y = glm::clamp(closest.y, r->_o.y - r->_h / 2, r->_o.y + r->_h / 2);

    // will not change o if inside aabb, so we must clamp further to the edges
    if (closest == o_) {
        glm::vec2 o_in_r = o_ - r->_o;
        if (std::abs(o_in_r.x) > std::abs(o_in_r.y)) {
            closest.x = r->_o.x + (o_in_r.x > 0 ? +r->_w : -r->_w) / 2;
        } else {
            closest.y = r->_o.y + (o_in_r.y > 0 ? +r->_h : -r->_h) / 2;
        }
    }

    return closest;
}

bool BVH::is_leaf() { return right == nullptr; }

size_t BVH::size() { return size_; }

void BVH::construct_(
    std::vector<Entity*> objects,
    std::vector<Index> sorted_x,
    std::vector<Index> sorted_z
) {
    assert(sorted_x.size() == sorted_z.size());
    assert(sorted_x.size() >= 1);
    size_ = sorted_x.size();
    if (sorted_x.size() == static_cast<size_t>(1)) {
        assert(sorted_x[0].obj == sorted_z[0].obj);
        right = nullptr;
        o = objects[sorted_x[0].obj];
        return;
    }

    // create bounding volume for this level, remembering the min/max in each
    // dim
    float min_x, max_x, min_z, max_z;
    min_x = min_z = std::numeric_limits<float>::max();
    max_x = max_z = -std::numeric_limits<float>::max();
    // tightly fit min_x/max_x/min_z/max_z
    for (size_t i = 0; i < sorted_x.size(); i++) {
        auto& bv = **POOL.get<BoundVolume*>(*objects[sorted_x[i].obj]);
        float dim_x, dim_z;
        if (bv._vt == BoundVolume::volume_type::RECT) {
            // add a nudge if you want fat BVs
            dim_x = static_cast<Rect*>(&bv)->_w / 2;
            dim_z = static_cast<Rect*>(&bv)->_h / 2;
        } else {
            dim_x = dim_z = static_cast<Circ*>(&bv)->_r;
        }

        float x_lo = bv._o.x - dim_x;
        float x_hi = bv._o.x + dim_x;
        float z_lo = bv._o.y - dim_z;
        float z_hi = bv._o.y + dim_z;
        min_x = std::min(min_x, x_lo);
        max_x = std::max(max_x, x_hi);
        min_z = std::min(min_z, z_lo);
        max_z = std::max(max_z, z_hi);
    }

    float dx = max_x - min_x;
    float dz = max_z - min_z;
    aabb = Rect(glm::vec2(min_x + dx / 2, min_z + dz / 2), dx, dz);

    // partition along longer axis, splitting indices equally
    std::vector<Index> x_rhs, x_lhs;
    std::vector<Index> z_rhs, z_lhs;
    if (dx > dz) {
        split_(sorted_x, sorted_z, x_lhs, z_lhs, x_rhs, z_rhs);
    } else {
        split_(sorted_z, sorted_x, z_lhs, x_lhs, z_rhs, x_rhs);
    }

    // probably the slowest part is the new, tbh XD
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
    std::vector<Index>& b_rhs
) {
    assert(sorted_a.size() == sorted_b.size());
    unsigned half =
        static_cast<unsigned>(static_cast<float>(sorted_a.size()) / 2.f);
    unsigned half_ceil =
        static_cast<unsigned>(static_cast<float>(sorted_a.size()) / 2.f + .5f);
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
    for ( //
        unsigned i = 0, lhs = 0, rhs = 0;
        i < static_cast<unsigned>(sorted_b.size());
        ++i
    ) {
        unsigned oth_i = sorted_b[i].oth;
        if (oth_i < half) {
            b_lhs[lhs] = Index(sorted_b[i].obj, oth_i);
            assert(oth_i < a_lhs.size());
            a_lhs[oth_i] = Index(a_lhs[oth_i].obj, lhs);
            ++lhs;
        } else {
            b_rhs[rhs] = Index(sorted_b[i].obj, oth_i - half);
            assert(oth_i - half < a_rhs.size());
            a_rhs[oth_i - half] = Index(a_rhs[oth_i - half].obj, rhs);
            ++rhs;
        }
    }
}
