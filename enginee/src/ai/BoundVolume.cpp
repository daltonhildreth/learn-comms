#include "BoundVolume.h"
#include <algorithm>
#include <array>

Rect::Rect(): BoundVolume(glm::vec2(0, 0), volume_type::RECT), _w(0), _h(0) {}
Rect::Rect(glm::vec2 o, float w, float h):
    BoundVolume(o, volume_type::RECT),
    _w(w),
    _h(h) {}

Circ::Circ(): BoundVolume(glm::vec2(0, 0), volume_type::CIRC), _r(0) {}
Circ::Circ(glm::vec2 o, float r): BoundVolume(o, volume_type::CIRC), _r(r) {}

BoundVolume::~BoundVolume() {}
BoundVolume::BoundVolume(): _o(glm::vec2(0, 0)), _vt(volume_type::CIRC) {}
BoundVolume::BoundVolume(glm::vec2 o, volume_type vt): _o(o), _vt(vt) {}

std::vector<BoundVolume*> Circ::minkowski_sum(BoundVolume* bv) {
    if (bv->_vt == volume_type::CIRC) {
        return minkowski_sum_(static_cast<Circ*>(bv));
    }
    if (bv->_vt == volume_type::RECT) {
        return minkowski_sum_(static_cast<Rect*>(bv));
    }
    return std::vector<BoundVolume*>();
}
std::vector<BoundVolume*> Circ::minkowski_sum_(Circ* b) {
    std::vector<BoundVolume*> bv = {new Circ(b->_o, _r + b->_r)};
    return bv;
}
std::vector<BoundVolume*> Circ::minkowski_sum_(Rect* b) {
    std::vector<BoundVolume*> bv = {
        new Rect(b->_o, b->_w, 2 * _r + b->_h),
        new Rect(b->_o, 2 * _r + b->_w, b->_h),
        new Circ(glm::vec2(b->_o.x + b->_w / 2, b->_o.y + b->_h / 2), _r),
        new Circ(glm::vec2(b->_o.x - b->_w / 2, b->_o.y + b->_h / 2), _r),
        new Circ(glm::vec2(b->_o.x + b->_w / 2, b->_o.y - b->_h / 2), _r),
        new Circ(glm::vec2(b->_o.x - b->_w / 2, b->_o.y - b->_h / 2), _r),
    };
    return bv;
}

std::vector<BoundVolume*> Rect::minkowski_sum(BoundVolume* bv) {
    if (bv->_vt == volume_type::CIRC) {
        return minkowski_sum_(static_cast<Circ*>(bv));
    }
    if (bv->_vt == volume_type::RECT) {
        return minkowski_sum_(static_cast<Rect*>(bv));
    }
    return std::vector<BoundVolume*>();
}

std::vector<BoundVolume*> Rect::minkowski_sum_(Rect* b) {
    std::vector<BoundVolume*> bv = {new Rect(b->_o, _w + b->_w, _h + b->_h)};
    return bv;
}
std::vector<BoundVolume*> Rect::minkowski_sum_(Circ* b) {
    std::vector<BoundVolume*> bv = {
        new Rect(b->_o, _w, 2 * b->_r + _h),
        new Rect(b->_o, 2 * b->_r + _w, _h),
        new Circ(glm::vec2(b->_o.x + _w / 2, b->_o.y + _h / 2), b->_r),
        new Circ(glm::vec2(b->_o.x - _w / 2, b->_o.y + _h / 2), b->_r),
        new Circ(glm::vec2(b->_o.x + _w / 2, b->_o.y - _h / 2), b->_r),
        new Circ(glm::vec2(b->_o.x - _w / 2, b->_o.y - _h / 2), b->_r),
    };
    return bv;
}

/// is p in circ?
bool Circ::collides(glm::vec2 p) {
    glm::vec2 diff = p - _o;
    return glm::dot(diff, diff) <= _r * _r;
}

/// does the line start + La_to_b * t {0<t<1} NOT intersect the circ?
/// Real-Time collision detection (p179) by Christer Ericson
bool Circ::line_of_sight(glm::vec2 start, glm::vec2 end) {
    glm::vec2 d = end - start;
    glm::vec2 m = start - _o;
    float b = glm::dot(m, d);
    float c = glm::dot(m, m) - _r * _r;

    // outside and pointing away
    if (c > 0.f && b > 0.f)
        return true; // MISS, has LoS

    float a = glm::dot(d, d);
    float discrim = b * b - a * c;
    if (discrim < 0.f)
        return true; // MISS, has LoS

    // hits circ, compute smallest t
    float t = (-b - sqrtf(discrim)) / a;

    if (t * t > a) // end of segment is before circ
        return true; // MISS, has LoS

    return false; // HIT, no LoS
}

/// where does the ray bo+v*t intersect the circ?
/// Real-Time Collision detection (p178) by Christer Ericson
float Circ::intersect(glm::vec2 o, glm::vec2 d) {
    glm::vec2 m = o - _o;
    float a = glm::dot(d, d);
    float b = glm::dot(m, d);
    float c = glm::dot(m, m) - _r * _r;

    // outside and pointing away
    if (c > 0.f && b > 0.f)
        return std::numeric_limits<float>::max(); // MISS

    float discrim = b * b - a * c;
    if (discrim < 0.f)
        return std::numeric_limits<float>::max(); // MISS

    // hits circ, compute smallest t
    float t = (-b - sqrtf(discrim)) / a;

    if (t < 0.f) // inside sphere
        return 0.f;
    return t;
}

/// is p in rect?
bool Rect::collides(glm::vec2 p) {
    return fabs(p.x - _o.x) <= _w / 2 && fabs(p.y - _o.y) <= _h / 2;
}

/// does the line a + (b-a) * t {0<t<1} NOT intersect the rect?
/// largely taken from p183 of real-time collision detection by Christer Ericson
bool Rect::line_of_sight(glm::vec2 a, glm::vec2 b) {
    glm::vec2 rmin{_o.x - _w / 2, _o.y - _h / 2};
    glm::vec2 rmax{_o.x + _w / 2, _o.y + _h / 2};
    glm::vec2 e = rmax - rmin; // halflength extent
    glm::vec2 m = a + b - rmin - rmax; // midpoint
    glm::vec2 d = b - a; // halflength vector

    // Separating axes of world coordinates
    float adx = static_cast<float>(fabs(d.x));
    if (static_cast<float>(fabs(m.x)) > e.x + adx)
        return true; // MISS, has LoS
    float ady = static_cast<float>(fabs(d.y));
    if (static_cast<float>(fabs(m.y)) > e.y + ady)
        return true; // MISS, has Los

    // add an epsilon to deal with (nearly) parllel segments to axes
    adx += 0.0000001f;
    ady += 0.0000001f;

    // try edge-edge cross-products of separating axes
    if (static_cast<float>(fabs(m.x * d.y - m.y * d.x)) > e.x * ady + e.y * adx)
        return true; // MISS, has LoS

    return false; // no separating axis, HIT, no LoS
}

/// where does the ray bo+v*t intersect the rect?
/// largely taken from p181 of real-time collision detection by Christer Ericson
/// TODO: many of the same ray optimizations
float Rect::intersect(glm::vec2 bo, glm::vec2 v) {
    float tmin = 0.0f;
    float tmax = std::numeric_limits<float>::max(); // distance ray can travel
    glm::vec2 rmin{_o.x - _w / 2, _o.y - _h / 2};
    glm::vec2 rmax{_o.x + _w / 2, _o.y + _h / 2};

    for (int i = 0; i < 2; ++i) {
        if (fabs(v[i]) < 0.0000001f) {
            // parallel line
            if (bo[i] < rmin[i] || bo[i] > rmax[i]) {
                return std::numeric_limits<float>::max(); // MISS
            }
        } else {
            // what are the near and far plane of this dimension's slab
            float near = (rmin[i] - bo[i]) / v[i];
            float far = (rmax[i] - bo[i]) / v[i];
            if (near > far) {
                std::swap(near, far);
            }
            tmin = std::max(tmin, near);
            tmax = std::min(tmax, far);
            // if any slab intersection do not overlap, there is LoS
            if (tmin > tmax) {
                return std::numeric_limits<float>::max(); // MISS
            }
        }
    }
    return tmin; // HIT
}
