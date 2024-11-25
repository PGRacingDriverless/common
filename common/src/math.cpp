#include "common/math.hpp"

double calculate_distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

Vector2D::Vector2D(double start_x, double start_y, double end_x, double end_y) : start_x(start_x), start_y(start_y), end_x(end_x), end_y(end_y) {
    length = calculate_distance(start_x, start_y, end_x, end_y);
    dx = end_x - start_x;
    dy = end_y - start_y;
    normalized_x = dx / length;
    normalized_y = dy / length;
}

point Vector2D::end_point_extended_vector(double new_length){
    double x,y;
    x = start_x + new_length * normalized_x;
    y = start_y + new_length * normalized_y;
    return point(x,y);
}