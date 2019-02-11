#pragma once
#include <functional>
#include <optional>
#include <unordered_map>
#include <unordered_set>

typedef unsigned int NodeId;
typedef std::unordered_set<NodeId> Nodes;

template <class T> class Graph {
public:
    Graph();

    NodeId add_vertex(const T& v);
    void del_vertex(const NodeId& v);
    void add_dir_edge(const NodeId& v, const NodeId& u);
    void add_edge(const NodeId& v, const NodeId& u);

    Nodes& edges(const NodeId& v);
    std::optional<T> data(const NodeId& v);

    void for_vertex(std::function<void(NodeId)> f);
    void for_edge(std::function<void(NodeId, NodeId)> f);

    size_t vertex_num();
    size_t edge_num();

private:
    std::unordered_map<NodeId, T> _vertices;
    std::unordered_map<NodeId, Nodes> _edges;
    NodeId _idx_counter;
};

template <class T>
inline Graph<T>::Graph(): _vertices({}), _edges({}), _idx_counter(0) {}

template <class T> inline NodeId Graph<T>::add_vertex(const T& v) {
    _vertices[_idx_counter] = v;
    return _idx_counter++;
}

template <class T> inline void Graph<T>::del_vertex(const NodeId& v) {
    _vertices.erase(v);

    for (NodeId adj : edges(v)) {
        _edges[adj].erase(v);
    }
    _edges.erase(v);
}

template <class T>
inline void Graph<T>::add_dir_edge(const NodeId& v, const NodeId& u) {
    assert(_vertices.count(v) && _vertices.count(u));
    _edges[v].insert(u);
}

template <class T>
inline void Graph<T>::add_edge(const NodeId& v, const NodeId& u) {
    add_dir_edge(v, u);
    add_dir_edge(u, v);
}

template <class T> inline Nodes& Graph<T>::edges(const NodeId& v) {
    if (_edges.count(v)) {
        return _edges[v];
    } else {
        return *(new Nodes());
    } // return _edges.count(v) ? _edges[const_cast<NodeId>(v)] : {};
}

template <class T> inline std::optional<T> Graph<T>::data(const NodeId& v) {
    if (_vertices.count(v)) {
        return _vertices[v];
    } else {
        return {};
    } // return _vertices.count(v) ? _vertices[const_cast<NodeId>(v)] : {};
}

template <class T>
inline void Graph<T>::for_vertex(std::function<void(NodeId)> f) {
    for (const auto& vertex : _vertices) {
        f(vertex.first);
    }
}

template <class T>
inline void Graph<T>::for_edge(std::function<void(NodeId, NodeId)> f) {
    for (const auto& from : _edges) {
        for (const auto& to : from.second) {
            f(from.first, to);
        }
    }
}

template <class T> inline size_t Graph<T>::vertex_num() {
    return _vertices.size();
}

template <class T> inline size_t Graph<T>::edge_num() { return _edges.size(); }
