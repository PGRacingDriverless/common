#include "common/visualization.hpp"

namespace common::viz
{
    void set_marker_color(visualization_msgs::msg::Marker &marker, const common::viz::Color &color)
    {
        marker.color.r = color.r;
        marker.color.g = color.g;
        marker.color.b = color.b;
        marker.color.a = color.a;
    }

    visualization_msgs::msg::Marker create_rviz_cone_visualization_message(const std::string &name_space, const common::cones::Cone &cone, const int marker_count)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = rclcpp::Time();
        marker.ns = name_space;

        switch (cone.get_side())
        {
        case common::cones::Cone::TrackSide::INNER:
            marker.id = marker_count;
            break;
        case common::cones::Cone::TrackSide::OUTER:
            marker.id = 1000 - marker_count;
            break;
        default:
            break;
        }

        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = cone.get_x();
        marker.pose.position.y = cone.get_y();
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = rclcpp::Duration(0, 0);
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        switch (cone.get_color())
        {
        case common::cones::Cone::Color::YELLOW:
            set_marker_color(marker, common::viz::YELLOW);
            break;
        case common::cones::Cone::Color::BLUE:
            set_marker_color(marker, common::viz::BLUE);
            break;
        case common::cones::Cone::Color::ORANGE:
            set_marker_color(marker, common::viz::ORANGE);
            break;
        default:
            // RCLCPP_ERROR(package_class->get_logger(), "No color set for cone, we have a problem here");
            break;
        }

        return marker;
    }

    visualization_msgs::msg::Marker create_rviz_line_visualization_message(const std::string &name_space, const common::cones::Cone &cone1, const common::cones::Cone &cone2, const size_t marker_id)
    {
        visualization_msgs::msg::Marker marker_line;
        marker_line.header.frame_id = "/map";
        marker_line.header.stamp = rclcpp::Time();
        marker_line.ns = name_space;
        marker_line.id = 1000 + marker_id;
        marker_line.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker_line.action = visualization_msgs::msg::Marker::ADD;
        marker_line.lifetime = rclcpp::Duration(2, 0);

        if (cone1.get_side() == common::cones::Cone::TrackSide::INNER && cone2.get_side() == common::cones::Cone::TrackSide::INNER)
        {
            set_marker_color(marker_line, common::viz::YELLOW);
        }
        else if (cone1.get_side() == common::cones::Cone::TrackSide::OUTER && cone2.get_side() == common::cones::Cone::TrackSide::OUTER)
        {
            set_marker_color(marker_line, common::viz::BLUE);
        }
        else
        {
            set_marker_color(marker_line, common::viz::RED);
        }

        marker_line.scale.x = 0.1;

        marker_line.points.push_back(geometry_msgs::msg::Point(cone1));
        marker_line.points.push_back(geometry_msgs::msg::Point(cone2));

        return marker_line;
    }

    visualization_msgs::msg::Marker create_rviz_vector_visualization_message(const std::string &name_space, const double start_x, const double start_y, const double end_x, const double end_y, const size_t marker_id, const common::viz::Color &color)
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
        const std::string &name_space, const boost::geometry::model::polygon<point> &final_match_area, const size_t marker_id, const common::viz::Color &color)
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

        for (const auto &item : final_match_area.outer())
        {
            marker_polygon_area.points.push_back(geometry_msgs::msg::Point(item));
        }

        return marker_polygon_area;
    }

    visualization_msgs::msg::Marker create_circle(
        const float x,
        const float y,
        const float diameter,
        const std::string &frame_id,
        const common::viz::Color &color,
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
        const common::viz::Color &color,
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

        set_marker_color(marker, color);

        int32_t seconds = static_cast<int32_t>(marker_lifetime_s);
        uint32_t nanoseconds = static_cast<uint32_t>((marker_lifetime_s - seconds) * 1e9);
        marker.lifetime = rclcpp::Duration(seconds, nanoseconds);
    }

    // Takes a Cone object to create a label over the given cone
    // used in create_id_labels_for_cone_array()
    visualization_msgs::msg::Marker create_text_label_marker_from_cone(
        const common::cones::Cone &cone,
        const std::string &text,
        const std::string &name_space,
        const std::string &frame_id,
        const float marker_lifetime_s,
        const size_t marker_id,
        const float scale_x,
        const float scale_y,
        const float scale_z)
    {

        visualization_msgs::msg::Marker label;

        set_marker_parameters(
            label,
            common::viz::GRAY,
            name_space,
            frame_id,
            marker_lifetime_s,
            marker_id,
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
            visualization_msgs::msg::Marker::ADD,
            scale_x,
            scale_y,
            scale_z);

        label.text = text + " " + std::to_string(marker_id);

        label.pose.position.x = cone.get_x();
        label.pose.position.y = cone.get_y();
        label.pose.position.z = 1;

        return label;
    }

    // Takes a ConeArray and creates cubes in given coordinates of cones
    visualization_msgs::msg::Marker create_cube_list_from_cone_array(
        const common::cones::ConeArray &cone_array,
        const std::string &frame_id,
        const common::viz::Color &color,
        const std::string &name_space,
        const float marker_lifetime_s)
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

        for (common::cones::Cone cone : cone_array)
        {
            cone_marker_list.points.push_back(geometry_msgs::msg::Point(cone));
        }

        marker_id++;
        return cone_marker_list;
    }

    // Takes ConeArray and connects given cone positions with lines
    visualization_msgs::msg::Marker create_line_list_from_cone_array(
        const common::cones::ConeArray &cone_array,
        const std::string &frame_id,
        const common::viz::Color &color,
        const std::string &name_space,
        const float marker_lifetime_s)
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

        for (std::size_t i = 1; i < cone_array.size(); i++)
        {
            cone_marker_list.points.push_back(geometry_msgs::msg::Point(cone_array[i - 1]));
            cone_marker_list.points.push_back(geometry_msgs::msg::Point(cone_array[i]));
        }

        marker_id++;
        return cone_marker_list;
    }

    // Takes ConePairArray and connects all points beetween cone pairs
    visualization_msgs::msg::Marker create_line_list_connecting_cone_pair_array(
        common::cones::ConePairArray &cone_pair_array,
        const std::string &frame_id,
        const common::viz::Color &color,
        const std::string &name_space,
        const float marker_lifetime_s)
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

        for (std::size_t i = 0; i < cone_pair_array.size(); i++)
        {
            cone_marker_list.points.push_back(geometry_msgs::msg::Point(cone_pair_array[i].getInner()));
            cone_marker_list.points.push_back(geometry_msgs::msg::Point(cone_pair_array[i].getOuter()));
        }

        marker_id++;
        return cone_marker_list;
    }

    visualization_msgs::msg::Marker create_line_list_from_cone_graph(
        const common::cones::ConeGraph &cone_graph,
        const std::string frame_id,
        const common::viz::Color color,
        const std::string name_space,
        const float marker_lifetime_s)
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

        for (lemon::ListGraph::EdgeIt e(cone_graph.graph); e != lemon::INVALID; ++e)
        {
            auto source_vertex = cone_graph.graph.u(e);
            auto target_vertex = cone_graph.graph.v(e);

            const common::cones::Cone &cone1 = cone_graph.cones[source_vertex];
            const common::cones::Cone &cone2 = cone_graph.cones[target_vertex];

            line_list.points.push_back(geometry_msgs::msg::Point(cone1));
            line_list.points.push_back(geometry_msgs::msg::Point(cone2));
        }

        marker_id++;
        return line_list;
    }
};