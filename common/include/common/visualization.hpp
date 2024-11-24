#ifndef COMMON_VISUALIZATION_HPP
#define COMMON_VISUALIZATION_HPP

#include "common/math.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "cones/cone_array.hpp"
#include "cones/cone_pair_array.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <boost/geometry/geometries/geometries.hpp>

#include <boost/graph/adjacency_list.hpp>

namespace common {
    typedef struct Color {
        float r;
        float g;
        float b;
        float a;
    } Color;
    constexpr common::Color BLUE = {0.0, 0.0, 1.0, 1.0};
    constexpr common::Color LIGHT_BLUE = {0.7, 0.7, 1.0, 1.0};
    constexpr common::Color BLUEISH = {0.3, 0.3, 0.8, 1.0};
    constexpr common::Color YELLOW = {1.0, 1.0, 0.0, 1.0};
    constexpr common::Color LIGHT_YELLOW = {1.0, 1.0, 0.7, 1.0};
    constexpr common::Color YELLOWISH = {0.8, 0.8, 0.3, 1.0};
    constexpr common::Color ORANGE = {1.0, 0.5, 0.0, 1.0};
    constexpr common::Color RED = {1.0, 0.0, 0.0, 1.0};
    constexpr common::Color GRAY = {0.5, 0.5, 0.5, 1.0};
};

using ConeGraph = boost::adjacency_list<
    boost::listS, boost::vecS, boost::undirectedS, boost::no_property,
    boost::property<boost::edge_weight_t, double>>;

[[deprecated("This is not recommended, please use ***_LIST for marker publishing")]]
visualization_msgs::msg::Marker create_rviz_cone_visualization_message(std::string name_space, const Cone& cone, int marker_count);

[[deprecated("This is not recommended, please use ***_LIST for marker publishing")]]
visualization_msgs::msg::Marker create_rviz_line_visualization_message(std::string name_space, const Cone& cone_inner, const Cone& cone_outer, size_t marker_id);

[[deprecated("This is not recommended, please use ***_LIST for marker publishing")]]
visualization_msgs::msg::Marker create_rviz_vector_visualization_message(std::string name_space, double start_x, double start_y, double end_x, double end_y, size_t marker_id, common::Color color);

[[deprecated("This is not recommended, please use ***_LIST for marker publishing")]]
visualization_msgs::msg::Marker create_rviz_polygon_visualization_message(std::string name_space, boost::geometry::model::polygon<point> final_match_area, size_t marker_id, common::Color color);

[[deprecated("We are not using it anymore")]]
void publish_rviz_visualization_message(rclcpp::Node* package_class,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher, const visualization_msgs::msg::Marker marker);

void set_marker_color(visualization_msgs::msg::Marker& marker, common::Color color);

visualization_msgs::msg::MarkerArray create_id_labels_for_cone_array(
    ConeArray &cone_array,
    const std::string text,
    const std::string frame_id,
    const std::string name_space = "default_labels",
    const float marker_lifetime_s = 2.0);

visualization_msgs::msg::Marker create_cube_list_from_cone_array(
    ConeArray &cone_array, 
    const std::string frame_id,
    const common::Color color = common::BLUE,
    const std::string name_space = "default_cubes",
    const float marker_lifetime_s = 2.0);

visualization_msgs::msg::Marker create_line_list_from_cone_array(
    ConeArray &cone_array, 
    const std::string frame_id,
    const common::Color color = common::RED,
    const std::string name_space = "default_lines",
    const float marker_lifetime_s = 2.0);

visualization_msgs::msg::Marker create_line_list_connecting_cone_pair_array(
    ConePairArray &cone_pair_array,
    const std::string frame_id,
    const common::Color color = common::RED,
    const std::string name_space = "default_connecting_lines",
    const float marker_lifetime_s = 2.0);
    
visualization_msgs::msg::Marker create_line_list_from_cone_graph(
    const ConeGraph& cone_graph,
    const ConeArray& cone_array,
    const std::string frame_id,
    const common::Color color = common::YELLOW,
    const std::string name_space  = "default_graph_lines",
    const float marker_lifetime_s = 2.0);

[[deprecated("We are using diffrent way of publishing")]]
void publish_cone_graph_as_lines(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher,
    const ConeGraph& cone_graph,
    const ConeArray& cone_array,
    const common::Color color,
    const std::string name_space,
    const std::string frame_id,
    const float marker_lifetime_s = 2.0);

[[deprecated("We are using diffrent way of publishing")]]
void publish_circle(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher,
    const float x,
    const float y,
    const float diameter,
    const common::Color color,
    const std::string name_space,
    const std::string frame_id,
    const float marker_lifetime_s = 2.0);

visualization_msgs::msg::Marker create_circle(
    const float x,
    const float y,
    const float diameter,
    const std::string frame_id,
    const common::Color color = common::BLUE,
    const std::string name_space = "default_circle",
    const float marker_lifetime_s = 2.0);

#endif