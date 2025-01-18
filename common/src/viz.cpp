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

visualization_msgs::msg::Marker create_rviz_vector_visualization_message(const std::string &name_space,
                                                                         const double start_x,
                                                                         const double start_y,
                                                                         const double end_x,
                                                                         const double end_y,
                                                                         const size_t marker_id,
                                                                         const color_t &color)
{
    // Config marker to display vector in RViz2
    visualization_msgs::msg::Marker direction_vector;
    direction_vector.header.stamp = rclcpp::Time();
    direction_vector.header.frame_id = "/map";
    direction_vector.ns = name_space;
    direction_vector.id = marker_id;
    direction_vector.type = visualization_msgs::msg::Marker::ARROW;
    direction_vector.action = visualization_msgs::msg::Marker::ADD;
    direction_vector.lifetime = rclcpp::Duration(2, 0);

    geometry_msgs::msg::Point start_point;
    start_point.x = start_x;
    start_point.y = start_y;
    start_point.z = 0;

    geometry_msgs::msg::Point end_point;
    end_point.x = end_x;
    end_point.y = end_y;
    end_point.z = 0;

    direction_vector.points.push_back(start_point);
    direction_vector.points.push_back(end_point);

    direction_vector.scale.x = 0.2; // Shaft diameter
    direction_vector.scale.y = 0.4; // Head diameter
    direction_vector.scale.z = 0.4; // Head length

    set_marker_color(direction_vector, color);

    return direction_vector;
}

visualization_msgs::msg::Marker create_rviz_polygon_visualization_message(
    const std::string &name_space,
    const std::vector<geometry_msgs::msg::Point> &final_match_area, // random placeholder instead of boost
    const size_t marker_id,
    const color_t &color)
{

    // Config marker to display final contour in RViz2
    visualization_msgs::msg::Marker marker_polygon_area;
    marker_polygon_area.header.stamp = rclcpp::Time();
    marker_polygon_area.header.frame_id = "/map";
    marker_polygon_area.ns = name_space;
    marker_polygon_area.id = marker_id; // Adjust id as needed
    marker_polygon_area.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_polygon_area.action = visualization_msgs::msg::Marker::ADD;
    marker_polygon_area.scale.x = 0.1;
    marker_polygon_area.lifetime = rclcpp::Duration(2, 0);

    set_marker_color(marker_polygon_area, color);
    marker_polygon_area.color.a = 0.5;

    geometry_msgs::msg::Point polygon_point;
    for (const auto &item : final_match_area) // .outer()
    {
        polygon_point.x = item.x;
        polygon_point.y = item.y;
        polygon_point.z = 0.0;
        marker_polygon_area.points.push_back(polygon_point);
    }

    return marker_polygon_area;
}

visualization_msgs::msg::Marker create_circle(
    const float x,
    const float y,
    const float diameter,
    const std::string &frame_id,
    const color_t &color,
    const std::string &name_space,
    const float marker_lifetime_s)
{

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
    const float scale_z)
{

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
    const float marker_lifetime_s )
{
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
    const float marker_lifetime_s )
{

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
        1, 1, 1);

    for (Cone cone : cones)
    {
        cone_marker_list.points.push_back(create_point(cone.position.x, cone.position.y, 0.0));
    }

    marker_id++;
    return cone_marker_list;
}
