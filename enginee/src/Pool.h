#ifndef POOL_H
#define POOL_H

#include "Entity.h"
#include "Transform.h"
#include "ai/Agent.h"
#include "ai/BoundVolume.h"
#include "ai/Dynamics.h"
#include "comms/CommComp.h"
#include "model/Mesh.h"
#ifdef WIN32
    #include <optional>
#else
    #include <experimental/optional>
#endif
#include <functional>
//TODO: get rid of unordered_map here, to rushed to do it right right now.
#include <unordered_map>
#include <cstdint>

template <typename T>
class DataTable {
public:
    DataTable();
    uint16_t create(T&&);
    T* get(uint16_t);
    void attach(uint16_t id, uint16_t e);
    #ifdef WIN32
    std::optional<uint16_t> other(uint16_t);
    #else
    std::experimental::optional<uint16_t> other(uint16_t);
    #endif
    void for_(std::function<void(T&, uint16_t)>);

    //TODO: get rid of unordered map
    std::unordered_map<uint16_t, std::pair<uint16_t, T>> _table;
    uint16_t _count;
};

class Pool {
public:
    Pool();

    Entity& spawn_entity();
    void for_entity(std::function<void(Entity&)>);
    //fix discrepancies between components for all entities
    void all_sync();

    //T must be a component type
    template <typename T>
    uint16_t create(T&&);

    //T must be a component type; also comp_id is NOT Entity::id
    template <typename T>
    T* get(uint16_t comp_id);

    //T must be a component type
    template <typename T>
    T* get(const Entity&);

    //T must be a component type
    template <typename T>
    void attach(Entity&, uint16_t);

    template <typename T>
    void for_(std::function<void(T&, Entity&)>);
private:
    std::unordered_map<uint16_t, Entity> _entity_table;
    uint16_t _entity_count;

    template<typename T>
    DataTable<T>& _table();

    template<typename T>
    unsigned _comp();

    //A 0 entry = 'null' entity
    DataTable<Transform> _transform_table;
    DataTable<Mesh> _mesh_table;
    DataTable<Dynamics> _dynamics_table;
    DataTable<BoundVolume*> _bound_volume_table;
    DataTable<Agent> _agent_table;
    DataTable<CommComp> _comm_table;
};

extern Pool POOL;

#endif//POOL_H
