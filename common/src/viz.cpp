#include "common/viz.hpp"

color_t create_color(float r, float g, float b, float a)
{
    color_t color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

visualization_msgs::msg::Marker create_circle(
    const float x,
    const float y,
    const float diameter,
    const std::string &frame_id,
    const color_t &color,
    const std::string &name_space,
    const float marker_lifetime_s
) {

    size_t marker_id = 0;

    // Config marker array to display a circle in RViz2
    visualization_msgs::msg::Marker circle_marker;
    circle_marker.header.frame_id = frame_id;
    circle_marker.header.stamp = rclcpp::Clock().now();
    circle_marker.ns = name_space;
    circle_marker.id = marker_id;
    circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    circle_marker.action = visualization_msgs::msg::Marker::ADD;
    circle_marker.scale.x = 0.075; // Line width
    circle_marker.color.r = color.r;
    circle_marker.color.g = color.g;
    circle_marker.color.b = color.b;
    circle_marker.color.a = color.a;
    int32_t seconds = static_cast<int32_t>(marker_lifetime_s);
    uint32_t nanoseconds = static_cast<uint32_t>((marker_lifetime_s - seconds) * 1e9);
    circle_marker.lifetime = rclcpp::Duration(seconds, nanoseconds);

    // Calculate points around the circumference of the circle
    const int num_points = 36; // Number of points to represent the circle
    float radius = diameter / 2.0;
    for (int i = 0; i <= num_points; ++i)
    {
        float angle = 2 * M_PI * i / num_points;
        geometry_msgs::msg::Point point;
        point.x = x + radius * cos(angle);
        point.y = y + radius * sin(angle);
        point.z = 0;
        circle_marker.points.push_back(point);
    }

    // Increment the marker ID for the next circle
    marker_id++;

    return circle_marker;
}

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
) {

    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = name_space;
    marker.id = marker_id;
    marker.type = type;
    marker.action = action;

    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;

    marker.color = color;

    int32_t seconds = static_cast<int32_t>(marker_lifetime_s);
    uint32_t nanoseconds = static_cast<uint32_t>((marker_lifetime_s - seconds) * 1e9);
    marker.lifetime = rclcpp::Duration(seconds, nanoseconds);
}

visualization_msgs::msg::Marker create_line_list_connecting(
    const std::vector<ConePair> &cone_pairs,
    const std::string &frame_id,
    const color_t &color,
    const std::string &name_space,
    const float marker_lifetime_s 
) {
    std::size_t marker_id = 0;
    visualization_msgs::msg::Marker cone_marker_list;

    set_marker_parameters(
        cone_marker_list,
        color,
        name_space,
        frame_id,
        marker_lifetime_s,
        marker_id,
        visualization_msgs::msg::Marker::LINE_LIST,
        visualization_msgs::msg::Marker::ADD,
        0.05, 0.05, 0.05);

    for (std::size_t i = 0; i < cone_pairs.size(); i++)
    {
        cone_marker_list.points.push_back(create_point(cone_pairs[i].left.position.x, cone_pairs[i].left.position.y, 0.0));
        cone_marker_list.points.push_back(create_point(cone_pairs[i].right.position.x, cone_pairs[i].right.position.y, 0.0));
    }

    marker_id++;
    return cone_marker_list;
}

visualization_msgs::msg::Marker create_cube_list(
    const std::vector<Cone> &cones,
    const std::string &frame_id,
    const color_t &color,
    const std::string &name_space,
    const float marker_lifetime_s 
) {

    std::size_t marker_id = 0;
    visualization_msgs::msg::Marker cone_marker_list;

    set_marker_parameters(
        cone_marker_list,
        color,
        name_space,
        frame_id,
        marker_lifetime_s,
        marker_id,
        visualization_msgs::msg::Marker::CUBE_LIST,
        visualization_msgs::msg::Marker::ADD,
        0.5, 0.5, 0.5);

    for (Cone cone : cones)
    {
        cone_marker_list.points.push_back(create_point(cone.position.x, cone.position.y, 0.0));
    }

    marker_id++;
    return cone_marker_list;
}
