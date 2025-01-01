#ifndef COMMON_CONE_HPP
#define COMMON_CONE_HPP

#include "visualization_msgs/msg/marker.hpp"

namespace common::cones
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
        Cone(double x, double y);
        Cone(double x, double y, TrackSide side, Color color);

        double get_x() const;
        void set_x(double x);

        double get_y() const;
        void set_y(double y);

        void setPos(double x, double y);

        TrackSide get_side() const;
        void set_side(TrackSide side);

        Color get_color() const;
        void set_color(Color color);

        bool operator==(const Cone &cone) const;
        bool operator!=(const Cone &cone) const;

        explicit operator geometry_msgs::msg::Point() const;

    private:
        double m_x = 0;
        double m_y = 0;
        TrackSide m_side = INNER;
        Color m_color = YELLOW;
    };
};

#endif