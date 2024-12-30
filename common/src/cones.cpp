#include "common/cones.hpp"

namespace pgr
{
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

    ConePair::ConePair(const Cone &cone_outer, const Cone &cone_inner) : cone_outer(cone_outer), cone_inner(cone_inner) {}

    Cone ConePair::getOuter() const
    {
        return cone_outer;
    }
    void ConePair::setOuter(const Cone &cone_outer)
    {
        this->cone_outer = cone_outer;
    }

    Cone ConePair::getInner() const
    {
        return cone_inner;
    }
    void ConePair::setInner(const Cone &cone_inner)
    {
        this->cone_inner = cone_inner;
    }

    bool ConePair::operator==(const ConePair &cone_pair) const
    {
        return cone_pair.cone_inner == cone_inner && cone_pair.cone_outer == cone_outer;
    }
    bool ConePair::operator!=(const ConePair &cone_pair) const
    {
        return !(*this == cone_pair);
    }

    inline void separate_cone_sides(const pgr::ConeArray &input_cone_array, pgr::ConeArray &inner_cone_array, pgr::ConeArray &outer_cone_array)
    {
        for (auto &item : input_cone_array)
        {
            if (item.get_side() == pgr::Cone::TrackSide::OUTER)
            {
                outer_cone_array.push_back(item);
            }
            else if (item.get_side() == pgr::Cone::TrackSide::INNER)
            {
                inner_cone_array.push_back(item);
            }
        }
    }

    inline void separate_cone_sides_from_cone_pairs(const pgr::ConePairArray &cone_pair_array, pgr::ConeArray &inner_cone_array, pgr::ConeArray &outer_cone_array)
    {
        for (auto &item : cone_pair_array)
        {
            inner_cone_array.push_back(item.getInner());
            outer_cone_array.push_back(item.getOuter());
        }
    }
};