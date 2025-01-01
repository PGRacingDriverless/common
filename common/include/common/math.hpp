#ifndef COMMON_MATH_HPP
#define COMMON_MATH_HPP
#include <cmath>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

typedef double coordinate_type;
typedef boost::geometry::model::d2::point_xy<coordinate_type> point;

namespace common::math
{
    inline double calculate_distance(const double x1, const double y1, const double x2, const double y2);
}

#endif