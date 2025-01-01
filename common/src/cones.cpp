#include "common/cones.hpp"

namespace common::cones
{
    Cone::Cone(double x, double y) : m_x(x), m_y(y) {}

    Cone::Cone(double x, double y, TrackSide side, Color color) : m_x(x), m_y(y), m_side(side), m_color(color) {}

    double Cone::get_x() const
    {
        return m_x;
    }
    void Cone::set_x(double x)
    {
        m_x = x;
    }

    double Cone::get_y() const
    {
        return m_y;
    }
    void Cone::set_y(double y)
    {
        m_y = y;
    }

    void Cone::setPos(double x, double y)
    {
        m_x = x;
        m_y = y;
    }

    Cone::TrackSide Cone::get_side() const
    {
        return m_side;
    }
    void Cone::set_side(TrackSide side)
    {
        m_side = side;
    }

    Cone::Color Cone::get_color() const
    {
        return m_color;
    }
    void Cone::set_color(Color color)
    {
        m_color = color;
    }

    bool Cone::operator==(const Cone &cone) const
    {
        return cone.m_x == m_x &&
               cone.m_y == m_y &&
               cone.m_side == m_side &&
               cone.m_color == m_color;
    }
    bool Cone::operator!=(const Cone &cone) const
    {
        return !(*this == cone);
    }

    Cone::operator geometry_msgs::msg::Point() const
    {
        geometry_msgs::msg::Point point;
        point.x = m_x;
        point.y = m_y;
        point.z = 0;
        return point;
    }

    ConePair::ConePair(const Cone &cone_outer, const Cone &cone_inner) : cone_outer(cone_outer), cone_inner(cone_inner) {}

    Cone ConePair::getOuter() const
    {
        return cone_outer;
    }
    void ConePair::setOuter(const Cone &cone_outer)
    {
        this->cone_outer = cone_outer;
    }

    Cone ConePair::getInner() const
    {
        return cone_inner;
    }
    void ConePair::setInner(const Cone &cone_inner)
    {
        this->cone_inner = cone_inner;
    }

    bool ConePair::operator==(const ConePair &cone_pair) const
    {
        return cone_pair.cone_inner == cone_inner && cone_pair.cone_outer == cone_outer;
    }
    bool ConePair::operator!=(const ConePair &cone_pair) const
    {
        return !(*this == cone_pair);
    }

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