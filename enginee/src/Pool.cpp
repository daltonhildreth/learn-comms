#include "Pool.h"

Pool POOL;

template <typename T> DataTable<T>::DataTable(): _table(), _count(0) {}

template <typename T> uint16_t DataTable<T>::create(T&& t) {
    // nothing should ever be assigned 0.
    ++_count;
    _table.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_count),
        std::forward_as_tuple(0, std::move(t))
    );
    return _count;
}

template <typename T> T* DataTable<T>::get(uint16_t id) {
    auto v = _table.find(id);
    if (id == 0 || v == _table.end()) {
        return nullptr;
    }
    return &(v->second.second);
}

template <typename T> void DataTable<T>::attach(uint16_t id, uint16_t e) {
    _table.at(id).first = e;
}

template <typename T> std::optional<uint16_t> DataTable<T>::other(uint16_t id) {
    auto v = _table.find(id);
    if (id == 0 || v == _table.end()) {
        return nullopt;
    }
    return v->first;
}

template <typename T>
void DataTable<T>::for_(std::function<void(T&, uint16_t)> f) {
    for (auto& v : _table) {
        f(v.second.second, v.second.first);
    }
}

Pool::Pool():
    _entity_table(),
    _entity_count(0),
    _transform_table(),
    _mesh_table(),
    _dynamics_table(),
    _bound_volume_table(),
    _agent_table() {}

Entity& Pool::spawn_entity() {
    ++_entity_count;
    // This next line creates the Entity in place at _entity_count in
    // the _entity_table, while also returning a reference to the Entity
    // created.
    auto spawn = _entity_table.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_entity_count),
        std::forward_as_tuple(_entity_count)
    );
    return spawn.first->second;
}

// this WILL be slow. :(
void Pool::all_sync() {
    for_entity([this](const Entity& e) {
        auto t = get<Transform>(e);
        auto d = get<Dynamics>(e);
        auto bv = get<BoundVolume*>(e);
        auto a = get<Agent>(e);
        glm::vec2 p2d;
        if (d) {
            if (t) {
                t->set_pos(d->pos);
            }
            p2d = glm::vec2(d->pos.x, d->pos.z);
        } else if (t) {
            p2d = glm::vec2(t->pos().x, t->pos().z);
        }
        if (bv && (t || d)) {
            (*bv)->_o = p2d;
        }
        if (a && (t || d)) {
            a->start = p2d;
        }
    });
}

void Pool::for_entity(std::function<void(Entity&)> f) {
    for (auto& e : _entity_table) {
        f(e.second);
    }
}

// clang-format off
template <typename T> DataTable<T>& Pool::_table() { throw; }
template <> DataTable<Transform>& Pool::_table<Transform>() { return _transform_table; }
template <> DataTable<Mesh>& Pool::_table<Mesh>() { return _mesh_table; }
template <> DataTable<Dynamics>& Pool::_table<Dynamics>() { return _dynamics_table; }
template <> DataTable<BoundVolume*>& Pool::_table<BoundVolume*>() { return _bound_volume_table; }
template <> DataTable<Agent>& Pool::_table<Agent>() { return _agent_table; }
template <> DataTable<CommComp>& Pool::_table<CommComp>() { return _comm_table; }
// clang-format on

template <typename T> unsigned Pool::_comp() { throw; }
template <> unsigned Pool::_comp<Transform>() { return 0; }
template <> unsigned Pool::_comp<Mesh>() { return 1; }
template <> unsigned Pool::_comp<Dynamics>() { return 2; }
template <> unsigned Pool::_comp<BoundVolume*>() { return 3; }
template <> unsigned Pool::_comp<Agent>() { return 4; }
template <> unsigned Pool::_comp<CommComp>() { return 5; }

template <typename T> uint16_t Pool::create(T&& t) {
    return _table<T>().create(std::move(t));
}
template uint16_t Pool::create<Transform>(Transform&&);
template uint16_t Pool::create<Mesh>(Mesh&&);
template uint16_t Pool::create<Dynamics>(Dynamics&&);
template uint16_t Pool::create<BoundVolume*>(BoundVolume*&&);
template uint16_t Pool::create<Agent>(Agent&&);
template uint16_t Pool::create<CommComp>(CommComp&&);

template <typename T> T* Pool::get(uint16_t id) { return _table<T>().get(id); }
template Transform* Pool::get<Transform>(uint16_t);
template Mesh* Pool::get<Mesh>(uint16_t);
template Dynamics* Pool::get<Dynamics>(uint16_t);
template BoundVolume** Pool::get<BoundVolume*>(uint16_t);
template Agent* Pool::get<Agent>(uint16_t);
template CommComp* Pool::get<CommComp>(uint16_t);

template <typename T> T* Pool::get(const Entity& e) {
    return _table<T>().get(e.comp[_comp<T>()]);
}
template Transform* Pool::get<Transform>(const Entity&);
template Mesh* Pool::get<Mesh>(const Entity&);
template Dynamics* Pool::get<Dynamics>(const Entity&);
template BoundVolume** Pool::get<BoundVolume*>(const Entity&);
template Agent* Pool::get<Agent>(const Entity&);
template CommComp* Pool::get<CommComp>(const Entity&);

template <typename T> void Pool::attach(Entity& e, uint16_t id) {
    e.comp[_comp<T>()] = id;
    _table<T>().attach(id, e.id);
}
template void Pool::attach<Transform>(Entity& e, uint16_t);
template void Pool::attach<Mesh>(Entity& e, uint16_t);
template void Pool::attach<Dynamics>(Entity& e, uint16_t);
template void Pool::attach<BoundVolume*>(Entity& e, uint16_t);
template void Pool::attach<Agent>(Entity& e, uint16_t);
template void Pool::attach<CommComp>(Entity& e, uint16_t);

template <typename T> void Pool::for_(std::function<void(T&, Entity&)> f) {
    _table<T>().for_([&f, this](T& t, uint16_t eid) {
        f(t, _entity_table.at(eid));
    });
}

// clang-format off
template void Pool::for_<Transform>(std::function<void(Transform&, Entity&)>);
template void Pool::for_<Mesh>(std::function<void(Mesh&, Entity&)>);
template void Pool::for_<Dynamics>(std::function<void(Dynamics&, Entity&)>);
template void Pool::for_<BoundVolume*>(std::function<void(BoundVolume*&, Entity&)>);
template void Pool::for_<Agent>(std::function<void(Agent&, Entity&)>);
template void Pool::for_<CommComp>(std::function<void(CommComp&, Entity&)>);
// clang-format on
