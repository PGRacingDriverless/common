#ifndef COMMON_CONES_HPP
#define COMMON_CONES_HPP

#include <vector>

namespace pgr
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

        bool operator==(const Cone &cone) const;
        bool operator!=(const Cone &cone) const;

    private:
        double m_x = 0;
        double m_y = 0;
        TrackSide m_side = INNER;
        Color m_color = YELLOW;
    };

    class ConePair
    {
    public:
        ConePair() = default;
        ConePair(const Cone &cone_outer, const Cone &cone_inner);

        Cone getOuter() const;
        void setOuter(const Cone &cone_outer);

        Cone getInner() const;
        void setInner(const Cone &cone_inner);

        bool operator==(const ConePair &cone_pair) const;
        bool operator!=(const ConePair &cone_pair) const;

    private:
        Cone cone_outer;
        Cone cone_inner;
    };

    using ConeArray = std::vector<Cone>;
    using ConePairArray = std::vector<ConePair>;

    inline void separate_cone_sides(const ConeArray &input_cone_array, pgr::ConeArray &inner_cone_array, pgr::ConeArray &outer_cone_array);

    inline void separate_cone_sides_from_cone_pairs(const pgr::ConePairArray &cone_pair_array, pgr::ConeArray &inner_cone_array, pgr::ConeArray &outer_cone_array);
};

#endif