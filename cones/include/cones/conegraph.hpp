#ifndef COMMON_CONEGRAPH_HPP
#define COMMON_CONEGRAPH_HPP

#include <vector>

#include <lemon/list_graph.h>
#include <lemon/path.h>

#include "cones/cone.hpp"
#include "cones/conepair.hpp"

namespace common::cones
{
    std::vector<lemon::ListGraph::Node> get_adjacent_vertices(
        const lemon::ListGraph &g, lemon::ListGraph::Node u);

    struct ConeGraph
    {
        ConeGraph();
        ConeGraph(const ConeGraph &other);
        lemon::ListGraph::Node add_node(Cone cone);
        lemon::ListGraph::Edge add_edge(lemon::ListGraph::Node u, lemon::ListGraph::Node v, double weight);
        lemon::ListGraph graph;
        lemon::ListGraph::EdgeMap<double> edge_weight;
        lemon::ListGraph::NodeMap<Cone> cones;
        void operator=(const ConeGraph &rhs);
    };
};

#endif