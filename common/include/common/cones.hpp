#ifndef COMMON_CONES_HPP
#define COMMON_CONES_HPP

#include <vector>

namespace pgr::cones
{
    class Cone
    {
    public:
        enum TrackSide
        {
            OUTER,
            INNER
        };
        enum Color
        {
            BLUE,
            YELLOW,
            ORANGE
        };

        Cone() = default;
        Cone(double x, double y, TrackSide side, Color color);

        double get_x() const;
        void set_x(double x);

        double get_y() const;
        void set_y(double y);

        TrackSide get_side() const;
        void set_side(TrackSide side);

        Color get_color() const;
        void set_color(Color color);
    private:
        double m_x = 0;
        double m_y = 0;
        TrackSide m_side = INNER;
        Color m_color = YELLOW;
    };

    class ConeArray
    {
    public:
        ConeArray() = default;
        ConeArray(const ConeArray &other);
        ConeArray(ConeArray &&other);

        void add_cone(const Cone &cone);

        std::size_t size() const;

        void clear();

        const std::vector<Cone> &get_cones() const;
        void set_cones(const std::vector<Cone> &cones);

        Cone operator[](const std::size_t index) const;

        void operator=(const ConeArray &rhs);

        void operator=(ConeArray &&rhs);
    private:
        std::vector<Cone> m_cone_array;
    };

    struct ConePair
    {
        pgr::cones::Cone cone_outer;
        pgr::cones::Cone cone_inner;
    };

    class ConePairArray
    {
    public:
        ConePairArray() = default;

        ConePairArray(const ConePairArray &other);

        ConePairArray(ConePairArray &&other);

        void add_pair(const ConePair &pair);

        void set_pairs(const std::vector<ConePair> &pairs);

        void clear();

        std::size_t size() const;

        const std::vector<ConePair> &get_pairs() const;

        ConePair operator[](const std::size_t index) const;

        void operator=(const ConePairArray &rhs);

        void operator=(ConePairArray &&rhs);
    private:
        std::vector<ConePair> m_cone_pairs;
    };

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

#endif