#include "common/cones.hpp"

namespace pgr::cones
{
    Cone::Cone(double x, double y, TrackSide side, Color color) : m_x{x}, m_y{y}, m_side{side}, m_color{color} {}

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

    ConeArray::ConeArray(const ConeArray &other)
    {
        m_cone_array = other.m_cone_array;
    }

    ConeArray::ConeArray(ConeArray &&other)
    {
        m_cone_array = std::move(other.m_cone_array);
    }

    void ConeArray::add_cone(const Cone &cone)
    {
        m_cone_array.push_back(cone);
    }

    std::size_t ConeArray::size() const
    {
        return m_cone_array.size();
    }

    void ConeArray::clear()
    {
        m_cone_array.clear();
    }

    const std::vector<Cone> &ConeArray::get_cones() const // deprecated
    {
        return m_cone_array;
    }

    void ConeArray::set_cones(const std::vector<Cone> &cones) // deprecated
    {
        m_cone_array = cones;
    }

    Cone ConeArray::operator[](const std::size_t index) const
    {
        return m_cone_array[index];
    }

    void ConeArray::operator=(const ConeArray &rhs)
    {
        m_cone_array = rhs.m_cone_array;
    }

    void ConeArray::operator=(ConeArray &&rhs)
    {
        m_cone_array = std::move(rhs.m_cone_array);
    }

    ConePairArray::ConePairArray(const ConePairArray &other)
    {
        m_cone_pairs = other.m_cone_pairs;
    }

    ConePairArray::ConePairArray(ConePairArray &&other)
    {
        m_cone_pairs = std::move(other.m_cone_pairs);
    }

    void ConePairArray::add_pair(const ConePair &pair)
    {
        m_cone_pairs.push_back(pair);
    }

    void ConePairArray::set_pairs(const std::vector<ConePair> &pairs)
    {
        m_cone_pairs = pairs;
    }

    void ConePairArray::clear()
    {
        m_cone_pairs.clear();
    }

    std::size_t ConePairArray::size() const
    {
        return m_cone_pairs.size();
    }

    const std::vector<ConePair> &ConePairArray::get_pairs() const
    {
        return m_cone_pairs;
    }

    ConePair ConePairArray::operator[](const std::size_t index) const
    {
        return m_cone_pairs[index];
    }

    void ConePairArray::operator=(const ConePairArray &rhs)
    {
        m_cone_pairs = rhs.m_cone_pairs;
    }

    void ConePairArray::operator=(ConePairArray &&rhs)
    {
        m_cone_pairs = std::move(rhs.m_cone_pairs);
    }

    inline void separate_cone_sides(const std::vector<pgr::cones::Cone> &input_cone_array, pgr::cones::ConeArray &inner_cone_array, pgr::cones::ConeArray &outer_cone_array)
    {
        for (auto &item : input_cone_array)
        {
            if (item.get_side() == pgr::cones::Cone::TrackSide::OUTER)
            {
                outer_cone_array.add_cone(item);
            }
            else if (item.get_side() == pgr::cones::Cone::TrackSide::INNER)
            {
                inner_cone_array.add_cone(item);
            }
        }
    }

    inline void separate_cone_sides_from_cone_pairs(const pgr::cones::ConePairArray &cone_pair_array, pgr::cones::ConeArray &inner_cone_array, pgr::cones::ConeArray &outer_cone_array)
    {
        for (auto &item : cone_pair_array.get_pairs())
        {
            inner_cone_array.add_cone(item.cone_inner);
            outer_cone_array.add_cone(item.cone_outer);
        }
    }

    inline bool check_pairs_equality(const pgr::cones::ConePair &pair1, const pgr::cones::ConePair &pair2)
    {
        bool inner_cone_equality = false;
        bool outer_cone_equality = false;

        if ((pair1.cone_inner.get_x() == pair2.cone_inner.get_x()) && (pair1.cone_inner.get_y() == pair2.cone_inner.get_y()))
        {
            inner_cone_equality = true;
        }

        if ((pair1.cone_outer.get_x() == pair2.cone_outer.get_x()) && (pair1.cone_outer.get_y() == pair2.cone_outer.get_y()))
        {
            outer_cone_equality = true;
        }

        return inner_cone_equality && outer_cone_equality;
    }
};