#include "cones/conegraph.hpp"

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

    visualization_msgs::msg::Marker ConeGraph::create_line_list(
        const std::string frame_id,
        const common::viz::Color color,
        const std::string name_space,
        const float marker_lifetime_s) const
    {

        static size_t marker_id = 0;

        visualization_msgs::msg::Marker line_list;

        set_marker_parameters(
            line_list,
            color,
            name_space,
            frame_id,
            marker_lifetime_s,
            marker_id,
            visualization_msgs::msg::Marker::LINE_LIST,
            visualization_msgs::msg::Marker::ADD,
            0.075, 0.075, 0.075); // not sure if those scale parameters are set correctly since originaly only scale.x was set

        for (lemon::ListGraph::EdgeIt e(this->graph); e != lemon::INVALID; ++e)
        {
            auto source_vertex = graph.u(e);
            auto target_vertex = graph.v(e);

            const common::cones::Cone &cone1 = cones[source_vertex];
            const common::cones::Cone &cone2 = cones[target_vertex];

            line_list.points.push_back(geometry_msgs::msg::Point(cone1));
            line_list.points.push_back(geometry_msgs::msg::Point(cone2));
        }

        marker_id++;
        return line_list;
    }
};