#include "cones/cone_graph.hpp"

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
    : graph_(), edge_weight_(graph_), cones_(graph_) {}

ConeGraph::ConeGraph(const ConeGraph &other)
    : graph_(), edge_weight_(graph_), cones_(graph_)
{
    lemon::GraphCopy<lemon::ListGraph, lemon::ListGraph> copy(other.graph_, graph_);
    copy.nodeMap(other.cones_, cones_);
    copy.edgeMap(other.edge_weight_, edge_weight_);
    copy.run();
}

lemon::ListGraph::Node ConeGraph::add_node(Cone cone)
{
    lemon::ListGraph::Node node = graph_.addNode();
    cones_[node] = cone;
    return node;
}

lemon::ListGraph::Edge ConeGraph::add_edge(lemon::ListGraph::Node u, lemon::ListGraph::Node v, double weight)
{
    lemon::ListGraph::Edge edge = graph_.addEdge(u, v);
    edge_weight_[edge] = weight;
    return edge;
}

void ConeGraph::operator=(const ConeGraph &rhs)
{
    lemon::GraphCopy<lemon::ListGraph, lemon::ListGraph> copy(rhs.graph_, graph_);
    copy.nodeMap(rhs.cones_, cones_);
    copy.edgeMap(rhs.edge_weight_, edge_weight_);
    copy.run();
}

visualization_msgs::msg::Marker ConeGraph::create_line_list(
    const std::string frame_id,
    const color_t color,
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

    for (lemon::ListGraph::EdgeIt e(graph_); e != lemon::INVALID; ++e)
    {
        auto source_vertex = graph_.u(e);
        auto target_vertex = graph_.v(e);

        const Cone &cone1 = cones_[source_vertex];
        const Cone &cone2 = cones_[target_vertex];

        line_list.points.push_back(geometry_msgs::msg::Point(cone1));
        line_list.points.push_back(geometry_msgs::msg::Point(cone2));
    }

    marker_id++;
    return line_list;
}

lemon::ListGraph &ConeGraph::graph()
{
    return graph_;
}
const lemon::ListGraph &ConeGraph::graph() const
{
    return graph_;
}

lemon::ListGraph::EdgeMap<double> &ConeGraph::edge_weight()
{
    return edge_weight_;
}
const lemon::ListGraph::EdgeMap<double> &ConeGraph::edge_weight() const
{
    return edge_weight_;
}

lemon::ListGraph::NodeMap<Cone> &ConeGraph::cones()
{
    return cones_;
}
const lemon::ListGraph::NodeMap<Cone> &ConeGraph::cones() const
{
    return cones_;
}

ConeGraph ConeGraph::create_neighborhood_graph(const ConeArray &cone_array, float neighborhood_distance)
{
    ConeGraph g;
    for (Cone cone : cone_array)
    {
        g.add_node(cone);
    }
    for (lemon::ListGraph::NodeIt u(g.graph_); u != lemon::INVALID; ++u)
    {
        for (lemon::ListGraph::NodeIt v(g.graph_); v != lemon::INVALID; ++v)
        {
            if (u != v)
            {
                double distance = dist(
                    g.cones_[u].get_x(), g.cones_[u].get_y(),
                    g.cones_[v].get_x(), g.cones_[v].get_y());
                if (distance < neighborhood_distance)
                {
                    g.add_edge(u, v, distance);
                }
            }
        }
    }

    return g;
}

ConeGraph::operator ConeArray() const
{
    ConeArray cone_array;

    for (lemon::ListGraph::NodeIt n(graph_); n != lemon::INVALID; ++n)
    {
        cone_array.push_back(cones_[n]);
    }

    std::reverse(cone_array.begin(), cone_array.end());

    return cone_array;
}