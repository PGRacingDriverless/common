#include "common/cone.hpp"

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
};