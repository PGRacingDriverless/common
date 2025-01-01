#ifndef COMMON_VISUALIZATION_HPP
#define COMMON_VISUALIZATION_HPP

#include "common/math.hpp"
#include "common/cones.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <boost/geometry/geometries/geometries.hpp>

#include <boost/graph/adjacency_list.hpp>

namespace common::viz
{
    struct Color
    {
        float r;
        float g;
        float b;
        float a;
    };
    constexpr Color BLUE = {0.0, 0.0, 1.0, 1.0};
    constexpr Color LIGHT_BLUE = {0.7, 0.7, 1.0, 1.0};
    constexpr Color BLUEISH = {0.3, 0.3, 0.8, 1.0};
    constexpr Color YELLOW = {1.0, 1.0, 0.0, 1.0};
    constexpr Color LIGHT_YELLOW = {1.0, 1.0, 0.7, 1.0};
    constexpr Color YELLOWISH = {0.8, 0.8, 0.3, 1.0};
    constexpr Color ORANGE = {1.0, 0.5, 0.0, 1.0};
    constexpr Color RED = {1.0, 0.0, 0.0, 1.0};
    constexpr Color GRAY = {0.5, 0.5, 0.5, 1.0};

    //using ConeGraph = boost::adjacency_list<
        //boost::listS, boost::vecS, boost::undirectedS, boost::no_property,
        //boost::property<boost::edge_weight_t, double>>;

    void set_marker_color(visualization_msgs::msg::Marker &marker, const common::viz::Color &color);

    [[deprecated("IN USE! - This is not recommended, please use ***_LIST for marker publishing")]]
    visualization_msgs::msg::Marker create_rviz_cone_visualization_message(const std::string &name_space, const pgr::Cone &cone, const int marker_count);
    
    [[deprecated("IN USE! - This is not recommended, please use ***_LIST for marker publishing")]]
    visualization_msgs::msg::Marker create_rviz_line_visualization_message(const std::string &name_space, const pgr::Cone &cone1, const pgr::Cone &cone2, const size_t marker_id);
    
    [[deprecated("IN USE! - This is not recommended, please use ***_LIST for marker publishing")]]
    visualization_msgs::msg::Marker create_rviz_vector_visualization_message(const std::string &name_space, const double start_x, const double start_y, const double end_x, const double end_y, const size_t marker_id, const common::viz::Color &color);
    
    [[deprecated("IN USE! - This is not recommended, please use ***_LIST for marker publishing")]]
    visualization_msgs::msg::Marker create_rviz_polygon_visualization_message(
        const std::string &name_space, const boost::geometry::model::polygon<point> &final_match_area, const size_t marker_id, const common::viz::Color &color);

    visualization_msgs::msg::Marker create_circle(
        const float x,
        const float y,
        const float diameter,
        const std::string &frame_id,
        const common::viz::Color &color,
        const std::string &name_space,
        const float marker_lifetime_s);

    // sets parameters for given marker
    void set_marker_parameters(
        visualization_msgs::msg::Marker &marker,
        const common::viz::Color &color,
        const std::string &name_space,
        const std::string &frame_id,
        const float marker_lifetime_s,
        const size_t marker_id,
        const int32_t type,
        const int32_t action,
        const float scale_x,
        const float scale_y,
        const float scale_z);

    // Takes a ConeArray and creates text label over cones with correspoding index/id
    visualization_msgs::msg::MarkerArray create_id_labels_for_cone_array(
        const pgr::ConeArray &cone_array,
        const std::string &frame_id,
        const std::string &text,
        const std::string &name_space,
        const float marker_lifetime_s);
    
    // Takes a ConeArray and creates cubes in given coordinates of cones
    visualization_msgs::msg::Marker create_cube_list_from_cone_array(
        const pgr::ConeArray &cone_array,
        const std::string &frame_id,
        const common::viz::Color &color,
        const std::string &name_space,
        const float marker_lifetime_s);
    
    // Takes ConeArray and connects given cone positions with lines
    visualization_msgs::msg::Marker create_line_list_from_cone_array(
        const pgr::ConeArray &cone_array,
        const std::string &frame_id,
        const common::viz::Color &color,
        const std::string &name_space,
        const float marker_lifetime_s);
    
    // Takes ConePairArray and connects all points beetween cone pairs
    visualization_msgs::msg::Marker create_line_list_connecting_cone_pair_array(
        pgr::ConePairArray &cone_pair_array,
        const std::string &frame_id,
        const common::viz::Color &color,
        const std::string &name_space,
        const float marker_lifetime_s);

    visualization_msgs::msg::Marker create_line_list_from_cone_graph(
        const pgr::ConeGraph &cone_graph,
        const std::string frame_id,
        const common::viz::Color color,
        const std::string name_space,
        const float marker_lifetime_s);
};

#endif