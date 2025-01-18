#ifndef COMMON_VISUALIZATION_HPP
#define COMMON_VISUALIZATION_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common/math.hpp"
#include "common/cone.hpp"
#include "common/msg.hpp"

typedef double coordinate_type;

using color_t = std_msgs::msg::ColorRGBA;

color_t create_color(float r, float g, float b, float a);

const color_t BLUE = create_color(0.0, 0.0, 1.0, 1.0);
const color_t LIGHT_BLUE = create_color(0.7, 0.7, 1.0, 1.0);
const color_t BLUEISH = create_color(0.3, 0.3, 0.8, 1.0);
const color_t YELLOW = create_color(1.0, 1.0, 0.0, 1.0);
const color_t LIGHT_YELLOW = create_color(1.0, 1.0, 0.7, 1.0);
const color_t YELLOWISH = create_color(0.8, 0.8, 0.3, 1.0);
const color_t ORANGE = create_color(1.0, 0.5, 0.0, 1.0);
const color_t RED = create_color(1.0, 0.0, 0.0, 1.0);
const color_t GRAY = create_color(0.5, 0.5, 0.5, 1.0);

void set_marker_color(visualization_msgs::msg::Marker &marker, const color_t &color);

visualization_msgs::msg::Marker create_circle(
    const float x,
    const float y,
    const float diameter,
    const std::string &frame_id,
    const color_t &color,
    const std::string &name_space,
    const float marker_lifetime_s
);

// sets parameters for given marker
void set_marker_parameters(
    visualization_msgs::msg::Marker &marker,
    const color_t &color,
    const std::string &name_space,
    const std::string &frame_id,
    const float marker_lifetime_s,
    const size_t marker_id,
    const int32_t type,
    const int32_t action,
    const float scale_x,
    const float scale_y,
    const float scale_z
);


visualization_msgs::msg::Marker create_line_list_connecting(
    const std::vector<ConePair> &cone_pairs,
    const std::string &frame_id,
    const color_t &color,
    const std::string &name_space,
    const float marker_lifetime_s
);

visualization_msgs::msg::Marker create_cube_list(
    const std::vector<Cone> &cones,
    const std::string &frame_id,
    const color_t &color,
    const std::string &name_space,
    const float marker_lifetime_s
);

#endif