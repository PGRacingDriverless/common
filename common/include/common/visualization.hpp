#ifndef COMMON_VISUALIZATION_HPP
#define COMMON_VISUALIZATION_HPP

#include "common/math.hpp"

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

    void set_marker_color(visualization_msgs::msg::Marker &marker, const common::viz::Color &color);
    
    visualization_msgs::msg::Marker create_rviz_vector_visualization_message(const std::string &name_space, const double start_x, const double start_y, const double end_x, const double end_y, const size_t marker_id, const common::viz::Color &color);
    
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
};

#endif