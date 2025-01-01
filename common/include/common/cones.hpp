#ifndef COMMON_CONES_HPP
#define COMMON_CONES_HPP

#include <vector>

#include <lemon/list_graph.h>
#include <lemon/path.h>

#include "cone.hpp"
#include "conepair.hpp"

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

    using ConeArray = std::vector<Cone>;
    using ConePairArray = std::vector<ConePair>;

    inline void separate_cone_sides(const ConeArray &input_cone_array, common::cones::ConeArray &inner_cone_array, common::cones::ConeArray &outer_cone_array);

    inline void separate_cone_sides_from_cone_pairs(const common::cones::ConePairArray &cone_pair_array, common::cones::ConeArray &inner_cone_array, common::cones::ConeArray &outer_cone_array);
};

#endif