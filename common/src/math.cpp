#include "common/math.hpp"

namespace pgr::math
{
    inline double calculate_distance(const double x1, const double y1, const double x2, const double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    Vector2D::Vector2D(const double start_x, const double start_y, const double end_x, const double end_y) : start_x(start_x), start_y(start_y), end_x(end_x), end_y(end_y)
    {
        length = calculate_distance(start_x, start_y, end_x, end_y);
        dx = end_x - start_x;
        dy = end_y - start_y;
        normalized_x = dx / length;
        normalized_y = dy / length;
    }

    inline point Vector2D::end_point_extended_vector(const double new_length)
    {
        return point(start_x + new_length * normalized_x, start_y + new_length * normalized_y);
    }
};