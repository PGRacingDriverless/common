#ifndef COMMON_MATH_HPP
#define COMMON_MATH_HPP
#include <cmath>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

typedef double coordinate_type;
typedef boost::geometry::model::d2::point_xy<coordinate_type> point;

namespace pgr::math
{
    inline double calculate_distance(const double x1, const double y1, const double x2, const double y2);

    struct Vector2D
    {
        double start_x;
        double start_y;
        double end_x;
        double end_y;
        double dx;
        double dy;
        double length;
        double normalized_x;
        double normalized_y;

        Vector2D(const double start_x, const double start_y, const double end_x, const double end_y);

        inline point end_point_extended_vector(const double new_length);
    };
};

#endif