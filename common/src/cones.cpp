#include "common/cones.hpp"

namespace common::cones
{
    std::vector<lemon::ListGraph::Node> get_adjacent_vertices(const lemon::ListGraph &g, lemon::ListGraph::Node u)
    {
        std::vector<lemon::ListGraph::Node> neighbors;
        for (lemon::ListGraph::IncEdgeIt e(g, u); e != lemon::INVALID; ++e)
        {
            neighbors.push_back(g.oppositeNode(u, e));
        }
        return neighbors;
    }

    ConeGraph::ConeGraph()
        : graph(), edge_weight(graph), cones(graph) {}

    ConeGraph::ConeGraph(const ConeGraph &other)
        : graph(), edge_weight(graph), cones(graph)
    {
        lemon::GraphCopy<lemon::ListGraph, lemon::ListGraph> copy(other.graph, graph);
        copy.nodeMap(other.cones, cones);
        copy.edgeMap(other.edge_weight, edge_weight);
        copy.run();
    }

    lemon::ListGraph::Node ConeGraph::add_node(Cone cone)
    {
        lemon::ListGraph::Node node = graph.addNode();
        cones[node] = cone;
        return node;
    }

    lemon::ListGraph::Edge ConeGraph::add_edge(lemon::ListGraph::Node u, lemon::ListGraph::Node v, double weight)
    {
        lemon::ListGraph::Edge edge = graph.addEdge(u, v);
        edge_weight[edge] = weight;
        return edge;
    }

    void ConeGraph::operator=(const ConeGraph &rhs)
    {
        lemon::GraphCopy<lemon::ListGraph, lemon::ListGraph> copy(rhs.graph, graph);
        copy.nodeMap(rhs.cones, cones);
        copy.edgeMap(rhs.edge_weight, edge_weight);
        copy.run();
    }

    inline void separate_cone_sides(const common::cones::ConeArray &input_cone_array, common::cones::ConeArray &inner_cone_array, common::cones::ConeArray &outer_cone_array)
    {
        for (auto &item : input_cone_array)
        {
            if (item.get_side() == common::cones::Cone::TrackSide::OUTER)
            {
                outer_cone_array.push_back(item);
            }
            else if (item.get_side() == common::cones::Cone::TrackSide::INNER)
            {
                inner_cone_array.push_back(item);
            }
        }
    }

    inline void separate_cone_sides_from_cone_pairs(const common::cones::ConePairArray &cone_pair_array, common::cones::ConeArray &inner_cone_array, common::cones::ConeArray &outer_cone_array)
    {
        for (auto &item : cone_pair_array)
        {
            inner_cone_array.push_back(item.getInner());
            outer_cone_array.push_back(item.getOuter());
        }
    }
};