#include "common/math.hpp"

double e_distance(const double x1, const double y1, const double x2, const double y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}