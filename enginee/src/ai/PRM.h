#pragma once
#include "../util/Graph.h"
#include "BoundVolume.h"
#include <glm/glm.hpp>
#include <memory>
#include <vector>

typedef std::unordered_set<glm::vec2> Points;
typedef std::vector<glm::vec2> PointPath;

// TODO: smart ownership
class Cspace2d {
public:
    // generates a configuartion space given a list of obstacles and agent
    Cspace2d(std::vector<BoundVolume*> obstacles, BoundVolume* agent);
    // detects if a point collides with anything in the configuration space
    bool collides(glm::vec2 a);
    // detects if a line segment between a and b collides with the C-space
    bool line_of_sight(glm::vec2 a, glm::vec2 b);

    ~Cspace2d();

    // TODO: unique_ptr
    std::vector<BoundVolume*> _obstacles;
};

class PRM {
public:
    // samples and connects a Pobabilistic Road Map
    PRM(std::unique_ptr<Cspace2d> cspace,
        float threshold = 5.f,
        float perturb = .2f,
        glm::vec2 bin_dim = glm::vec2(2.f, 2.f),
        int bin_samp = 1,
        glm::vec2 lo_bound = -glm::vec2(10.f, 10.f),
        glm::vec2 hi_bound = glm::vec2(10.f, 10.f),
        float variance = 1.f);

    std::unique_ptr<Graph<glm::vec2>> _roadmap;

private:
    // configuration/collision space
    std::unique_ptr<Cspace2d> _cspace;

    // distance threshold for connecting
    float _threshold;
    // sample perturbation to get every sample; 1.f = nudge at most one unit
    // Helps to put points near object edges
    float _perturb;
    // bin sampling dimensions
    glm::vec2 _bin_dim;
    // number of samples per bin
    int _bin_samp;
    // low end of the total dimensions
    glm::vec2 _lo_bound;
    // high end of the total dimensions
    glm::vec2 _hi_bound;
    // 1.f = 100% of bin; .2f = 20% of bin on the center. Sample in here.
    float _variance;

    // Uniformly samples the c-space to generate points for the roadmap.
    // All sampled points will not collide with the static environment
    void sample_space();
    // threshold search to find nearest neighbors
    std::unique_ptr<Nodes> nearby(NodeId source);
    // connects nearest neighbors by Graph edges.
    void connect();
};
