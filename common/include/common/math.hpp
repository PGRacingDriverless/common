#ifndef COMMON_MATH_HPP
#define COMMON_MATH_HPP
#include <cmath>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

typedef double coordinate_type;
typedef boost::geometry::model::d2::point_xy<coordinate_type> point;

struct Vector2D{
    double start_x;
    double start_y;
    double end_x;
    double end_y;
    double dx;
    double dy;
    double length;
    double normalized_x;
    double normalized_y;

    Vector2D(double start_x, double start_y, double end_x, double end_y);
    point end_point_extended_vector(double new_length);

};

double calculate_distance(double x1, double y1, double x2, double y2);

#endif