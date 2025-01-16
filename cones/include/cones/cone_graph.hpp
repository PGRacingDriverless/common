#ifndef COMMON_CONEGRAPH_HPP
#define COMMON_CONEGRAPH_HPP

#include <vector>
#include <algorithm>

#include <lemon/list_graph.h>
#include <lemon/path.h>

#include "cones/cone.hpp"
#include "cones/cone_pair.hpp"
#include "common/msg.hpp"

#include "common/math.hpp"

std::vector<lemon::ListGraph::Node> get_adjacent_vertices(
    const lemon::ListGraph &g, lemon::ListGraph::Node u);

class ConeGraph
{
private:
    lemon::ListGraph graph_;
    lemon::ListGraph::EdgeMap<double> edge_weight_;
    lemon::ListGraph::NodeMap<ConeNew> cones_;

public:
    ConeGraph();
    ConeGraph(const ConeGraph &other);

    ~ConeGraph() = default;

    lemon::ListGraph::Node add_node(ConeNew cone);
    lemon::ListGraph::Edge add_edge(lemon::ListGraph::Node u, lemon::ListGraph::Node v, double weight);

    void operator=(const ConeGraph &rhs);

    visualization_msgs::msg::Marker create_line_list(
        const std::string frame_id,
        const color_t color,
        const std::string name_space,
        const float marker_lifetime_s) const;

    lemon::ListGraph &graph();
    const lemon::ListGraph &graph() const;

    lemon::ListGraph::EdgeMap<double> &edge_weight();
    const lemon::ListGraph::EdgeMap<double> &edge_weight() const;

    lemon::ListGraph::NodeMap<ConeNew> &cones();
    const lemon::ListGraph::NodeMap<ConeNew> &cones() const;

    static ConeGraph create_neighborhood_graph(
        const std::vector<ConeNew> &cone_array,
        float neighborhood_distance
    );

    operator std::vector<ConeNew>() const;
};

#endif