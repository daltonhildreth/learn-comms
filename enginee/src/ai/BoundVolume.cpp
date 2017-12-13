#include "BoundVolume.h"
#include <array>
#include <algorithm>

Rect::Rect()
    : BoundVolume(glm::vec2(0, 0), volume_type::RECT), _w(0), _h(0) {}
Rect::Rect(glm::vec2 o, float w, float h)
    : BoundVolume(o, volume_type::RECT), _w(w), _h(h) {}

Circ::Circ()
    : BoundVolume(glm::vec2(0, 0), volume_type::CIRC), _r(0) {}
Circ::Circ(glm::vec2 o, float r)
    : BoundVolume(o, volume_type::CIRC), _r(r) {}

BoundVolume::~BoundVolume() {}
BoundVolume::BoundVolume() : _o(glm::vec2(0, 0)), _vt(volume_type::CIRC) {}
BoundVolume::BoundVolume(glm::vec2 o, volume_type vt) : _o(o), _vt(vt) {}

std::vector<BoundVolume*> Circ::minkowski_sum(BoundVolume* bv) {
    if (bv->_vt == volume_type::CIRC) {
        return minkowski_sum_(dynamic_cast<Circ*>(bv));
    }
    if (bv->_vt == volume_type::RECT) {
        return minkowski_sum_(dynamic_cast<Rect*>(bv));
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
        return minkowski_sum_(dynamic_cast<Circ*>(bv));
    }
    if (bv->_vt == volume_type::RECT) {
        return minkowski_sum_(dynamic_cast<Rect*>(bv));
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
        new Rect(b->_o, 2 *b->_r + _w, _h),
        new Circ(glm::vec2(b->_o.x + _w / 2, b->_o.y + _h / 2), b->_r),
        new Circ(glm::vec2(b->_o.x - _w / 2, b->_o.y + _h / 2), b->_r),
        new Circ(glm::vec2(b->_o.x + _w / 2, b->_o.y - _h / 2), b->_r),
        new Circ(glm::vec2(b->_o.x - _w / 2, b->_o.y - _h / 2), b->_r),
    };
    return bv;
}

bool Circ::collides(glm::vec2 p) {
    glm::vec2 diff = p - _o;
    return glm::dot(diff, diff) <= _r * _r;
}

bool Circ::line_of_sight(glm::vec2 a, glm::vec2 b, glm::vec2 Lab, float len2) {
    glm::vec2 Lbo = _o - b;
    float r2 = _r * _r;
    if (glm::dot(Lbo, Lbo) <= r2) //point b inside circle
        return false; // HIT

    glm::vec2 Lao = _o - a;
    //we don't use is_collision because we use a lot of these values again
    if (glm::dot(Lao, Lao) <= r2) //point a inside circle
        return false; // HIT

    float ang = glm::dot(Lab, Lao);
    glm::vec2 proj = Lab * (ang / len2);
    glm::vec2 rej = Lao - proj;
    float projlen2 = glm::dot(proj, proj);

    if (glm::dot(rej, rej) <= r2 //close enough tangentially
        && 0 <= ang              //point a before circle center
        && projlen2 <= len2)     //point b after circle center
        return false; // HIT

    return true; // else MISS
}

float Circ::intersect(glm::vec2 bo, glm::vec2 v) {
    //min t = - (u.v) +- sqrt((u.v)^. - (v.v) (u.u - r^))/(v.v)
    glm::vec2 u = bo - _o;
    float c = glm::dot(u, u) - _r * _r;
    if (c < 0)//inside
        return 0;

    float b = glm::dot(u, v);
    float a = glm::dot(v, v);

    float d = b*b - a*c;
    if (d <= 0)//no intersect, forward or backwards
        return std::numeric_limits<float>::max();

    float t = (-b - static_cast<float>(sqrt(d))) / a;
    if (t < 0)//intersction is behind ray
        return std::numeric_limits<float>::max();

    return t;
}

bool Rect::collides(glm::vec2 p) {
    return fabs(p.x - _o.x) <= _w / 2
        && fabs(p.y - _o.y) <= _h / 2;
}

static bool on_line(float t) {
    return 0 <= t && t <= 1;
}

static bool on_ray(float t) {
    return 0 <= t;
}

template <bool (f)(float)>
static float hit_clamp(float t) {
    return f(t) ? t : std::numeric_limits<float>::max();
}

static bool hit(float t) {
    return t < std::numeric_limits<float>::max();
}

//find (t, s) such that Lo + Ld*t = Mo + Md*s
static std::pair<float, float> line_intersect(glm::vec2 Lo, glm::vec2 Ld,
        glm::vec2 Mo, glm::vec2 Md) {
    glm::vec2 MLo = Mo - Lo;
    float s = (Ld.x*MLo.y - Ld.y*MLo.x) / (Md.x * Ld.y - Md.y * Ld.x);
    float t;
    if (fabs(Ld.x) < 0.00000001) {
        t = (Mo.y + Md.y * s - Lo.x) / Ld.y;
    } else {
        t = (Mo.x + Md.x * s - Lo.x) / Ld.x;
    }
    return {t, s};
}

bool Rect::line_of_sight(glm::vec2 start, glm::vec2 /*end*/,
        glm::vec2 La_to_b, float len2) {
    //dims
    float l = _o.x - _w / 2;
    float r = _o.x + _w / 2;
    float t = _o.y + _h / 2;
    float b = _o.y - _h / 2;
    //corners
    glm::vec2 bl(l, b);
    glm::vec2 br(r, b);
    glm::vec2 tl(l, t);
    glm::vec2 tr(r, t);

    std::array<std::pair<glm::vec2, glm::vec2>, 4> edges =
        {std::make_pair(tr, tl),
        std::make_pair(br, tr),
        std::make_pair(bl, br),
        std::make_pair(tl, bl)};

    return std::none_of(edges.cbegin(), edges.cend(),
        [start, La_to_b, len2](std::pair<glm::vec2, glm::vec2> edge) {
            auto&& result = line_intersect(start, La_to_b/sqrtf(len2),
                edge.second, edge.first - edge.second);
            return hit(hit_clamp<on_line>(result.second))
                && hit(hit_clamp<on_ray>(result.first));
        });
}

//assumes axis alignment
float Rect::intersect(glm::vec2 bo, glm::vec2 v) {
    if (collides(bo))
        return 0;

    //dims
    float l = _o.x - _w / 2;
    float r = _o.x + _w / 2;
    float t = _o.y + _h / 2;
    float b = _o.y - _h / 2;
    //corners
    glm::vec2 bl(l, b);
    glm::vec2 br(r, b);
    glm::vec2 tl(l, t);
    glm::vec2 tr(r, t);

    std::array<std::pair<glm::vec2, glm::vec2>, 4> edges =
        {std::make_pair(tr, tl),
        std::make_pair(br, tr),
        std::make_pair(bl, br),
        std::make_pair(tl, bl)};

    //find closest edge
    float p = std::numeric_limits<float>::max();
    for (auto& pair : edges) {
        auto&& edge = line_intersect(bo, v,
            pair.second, pair.first - pair.second);
        if (hit(hit_clamp<on_line>(edge.second))) {
            p = std::min(p, hit_clamp<on_ray>(edge.first));
        }
    }
    return p;
}
