#include "cones/cone.hpp"

namespace common::cones
{
    Cone::Cone(double x, double y) : m_x(x), m_y(y) {}

    Cone::Cone(double x, double y, TrackSide side, Color color) : m_x(x), m_y(y), m_side(side), m_color(color) {}

    Cone::Cone(const Cone& cone)
    {
        m_x = cone.m_x;
        m_y = cone.m_y;
        m_side = cone.m_side;
        m_color = cone.m_color;
    }

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

    Cone::operator common_msgs::msg::Cone() const
    {
        common_msgs::msg::Cone cone;

        cone.x = m_x;
        cone.y = m_y;
        cone.side = m_side;

        return cone;
    }

    visualization_msgs::msg::Marker Cone::create_rviz_visualization_message(const std::string &name_space, const int marker_count) const
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = rclcpp::Time();
        marker.ns = name_space;

        switch (m_side)
        {
        case common::cones::Cone::TrackSide::LEFT:
            marker.id = marker_count;
            break;
        case common::cones::Cone::TrackSide::RIGHT:
            marker.id = 1000 - marker_count;
            break;
        default:
            break;
        }

        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = m_x;
        marker.pose.position.y = m_y;
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = rclcpp::Duration(0, 0);
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        switch (m_color)
        {
        case common::cones::Cone::Color::YELLOW:
            marker.color = common::viz::YELLOW;
            break;
        case common::cones::Cone::Color::BLUE:
            marker.color = common::viz::BLUE;
            break;
        case common::cones::Cone::Color::ORANGE:
            marker.color = common::viz::ORANGE;
            break;
        default:
            // RCLCPP_ERROR(package_class->get_logger(), "No color set for cone, we have a problem here");
            break;
        }

        return marker;
    }

    visualization_msgs::msg::Marker Cone::create_rviz_line_visualization_message(const std::string &name_space, const common::cones::Cone &cone1, const common::cones::Cone &cone2, const size_t marker_id)
    {
        visualization_msgs::msg::Marker marker_line;
        marker_line.header.frame_id = "/map";
        marker_line.header.stamp = rclcpp::Time();
        marker_line.ns = name_space;
        marker_line.id = 1000 + marker_id;
        marker_line.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker_line.action = visualization_msgs::msg::Marker::ADD;
        marker_line.lifetime = rclcpp::Duration(2, 0);

        if (cone1.get_side() == common::cones::Cone::TrackSide::LEFT && cone2.get_side() == common::cones::Cone::TrackSide::LEFT)
        {
            marker_line.color = common::viz::YELLOW;
        }
        else if (cone1.get_side() == common::cones::Cone::TrackSide::RIGHT && cone2.get_side() == common::cones::Cone::TrackSide::RIGHT)
        {
            marker_line.color = common::viz::BLUE;
        }
        else
        {
            marker_line.color = common::viz::RED;
        }

        marker_line.scale.x = 0.1;

        marker_line.points.push_back(geometry_msgs::msg::Point(cone1));
        marker_line.points.push_back(geometry_msgs::msg::Point(cone2));

        return marker_line;
    }

    visualization_msgs::msg::Marker Cone::create_text_label_marker(
        const std::string &text,
        const std::string &name_space,
        const std::string &frame_id,
        const float marker_lifetime_s,
        const size_t marker_id,
        const float scale_x,
        const float scale_y,
        const float scale_z) const
    {

        visualization_msgs::msg::Marker label;

        common::viz::set_marker_parameters(
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

        label.pose.position.x = m_x;
        label.pose.position.y = m_y;
        label.pose.position.z = 1;

        return label;
    }
};