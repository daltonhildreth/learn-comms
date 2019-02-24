#include "PRM.h"
#include "../util/Seeder.h"
#include <algorithm>
#include <random>

using namespace std;

unique_ptr<Nodes> PRM::nearby(NodeId source) {
    unique_ptr<Nodes> neighbors(new Nodes());
    glm::vec2 from = *_roadmap->data(source);
    _roadmap->for_vertex([&](NodeId v) {
        glm::vec2 test = *_roadmap->data(v);
        // not the identical vertex and close enough
        if (v != source && glm::distance(from, test) < _threshold) {
            neighbors->insert(v);
        }
    });
    return neighbors;
}

void PRM::connect() {
    _roadmap->for_vertex([&](NodeId v) {
        // connect nearby nodes to v
        unsigned connected_to = 0;
        auto near = nearby(v);
        for (NodeId i : *near) {
            if (connected_to >= 8) break;
            // if the nearby points are within line of sight
            if ( //
                _cspace->line_of_sight(*_roadmap->data(v), *_roadmap->data(i))
            ) {
                // directed because we'll traverse the other side in for_vertex
                _roadmap->add_dir_edge(v, i);
                ++connected_to;
            }
        }
    });
}

void PRM::sample_space() {
    Seeder s;
    typedef uniform_real_distribution<float> UFD;
    UFD perturber(-_perturb, _perturb);
    UFD sampler_x(-_bin_dim.x * _variance / 2.f, _bin_dim.x * _variance / 2.f);
    UFD sampler_y(-_bin_dim.y * _variance / 2.f, _bin_dim.y * _variance / 2.f);

    // for each bin
    for (float x = _lo_bound.x; x < _hi_bound.x; x += _bin_dim.x) {
        for (float y = _lo_bound.y; y < _hi_bound.y; y += _bin_dim.y) {
            // for multiple samples per bin
            for (int i = 0; i < _bin_samp; ++i) {
                glm::vec2 sample(sampler_x(s.gen()), sampler_y(s.gen()));
                // place on correct bin
                sample += glm::vec2(x, y);
                // center point on bin
                sample += _bin_dim / 2.f;

                // nudge until out of something; could theoretically take a
                // while only do this if perturb is not 0.
                bool collides = false;
                while ( //
                    abs(_perturb) > 0 //
                    && (collides = _cspace->collides(sample))
                ) {
                    sample += glm::vec2(perturber(s.gen()), perturber(s.gen()));
                    sample = glm::clamp(sample, _lo_bound, _hi_bound);
                }

                // if it wasn't perturbed, need to check for collision.
                if (!collides) {
                    _roadmap->add_vertex(sample);
                }
            }
        }
    }

    // for each obstacle
    for (const auto& o : _cspace->_obstacles) {
        glm::vec2 h;
        glm::vec2 l;
        if (o->_vt == BoundVolume::volume_type::RECT) {
            auto r = static_cast<const Rect*>(o);
            h = {r->_o.x + r->_w/2 + _perturb, r->_o.y + r->_h/2 + _perturb};
            l = {r->_o.x - r->_w/2 - _perturb, r->_o.y - r->_h/2 - _perturb};
            printf("shit did happen %f %f / %f %f\n", h.x, h.y, l.x, l.y);
        } else {
            auto c = static_cast<const Circ*>(o);
            h = {c->_o.x + c->_r + _perturb, c->_o.y + c->_r + _perturb};
            l = {c->_o.x - c->_r - _perturb, c->_o.y - c->_r - _perturb};
        }

        glm::vec2 hh{h.x, h.y};
        glm::vec2 hl{h.x, l.y};
        glm::vec2 lh{l.x, h.y};
        glm::vec2 ll{l.x, l.y};
        for (const glm::vec2& p : {hh, hl, lh, ll}) {
            if (!_cspace->collides(p)) {
                _roadmap->add_vertex(p); 
            }
        }
    }
}

PRM::PRM(
    unique_ptr<Cspace2d> cspace,
    float threshold,
    float perturb,
    glm::vec2 bin_dim,
    int bin_samp,
    glm::vec2 lo_bound,
    glm::vec2 hi_bound,
    float variance
):
    _cspace(std::move(cspace)),
    _threshold(threshold),
    _perturb(perturb),
    _bin_dim(bin_dim),
    _bin_samp(bin_samp),
    _lo_bound(lo_bound),
    _hi_bound(hi_bound),
    _variance(variance) {
    _roadmap = std::make_unique<Graph<glm::vec2>>();
    sample_space();
    connect();
}

Cspace2d::Cspace2d(vector<BoundVolume*> obs, BoundVolume* agent) {
    _obstacles = vector<BoundVolume*>();

    for (BoundVolume* o : obs) {
        vector<BoundVolume*> ms = agent->minkowski_sum(o);
        _obstacles.insert(_obstacles.end(), ms.begin(), ms.end());
    }
}

Cspace2d::~Cspace2d() {
    for (BoundVolume* bv : _obstacles) {
        delete bv;
    }
}

bool Cspace2d::collides(glm::vec2 p) {
    for (BoundVolume* bv : _obstacles) {
        if (bv->collides(p)) {
            return true; // HIT
        }
    }
    return false; // MISS
}

bool Cspace2d::line_of_sight(glm::vec2 a, glm::vec2 b) {
    glm::vec2 Lab;
    Lab.x = b.x - a.x;
    Lab.y = b.y - a.y;
    float len2 = glm::dot(Lab, Lab);

    return std::all_of(_obstacles.begin(), _obstacles.end(), [&](auto* bv) {
        return bv->line_of_sight(a, b, Lab, len2);
    });
}
