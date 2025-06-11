#ifndef COMMON_MATH_HPP
#define COMMON_MATH_HPP

#include <pcl/point_types.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
#include <vector>
#include <stdexcept>

double dist(const double x1, const double y1, const double x2, const double y2);

pcl::PointXY calculate_direction_vector(const pcl::PointXY &p1, const pcl::PointXY &p2);
pcl::PointXY calculate_direction_vector(const geometry_msgs::msg::Pose &pose);

float quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q);

class CubicSplineInterpolator {
public:
    CubicSplineInterpolator(const std::vector<float>& t_coords,
                            const std::vector<float>& x_coords,
                            const std::vector<float>& y_coords);

    float interpolate_x(float t_val) const;
    float interpolate_y(float t_val) const;

private:
    std::vector<float> t, x, y;
    std::vector<float> ax, bx, cx, dx;
    std::vector<float> ay, by, cy, dy;

    void computeSplineCoefficients(const std::vector<float>& t,
                                   const std::vector<float>& values,
                                   std::vector<float>& a,
                                   std::vector<float>& b,
                                   std::vector<float>& c,
                                   std::vector<float>& d);

    float interpolate(float t_val,
                      const std::vector<float>& a,
                      const std::vector<float>& b,
                      const std::vector<float>& c,
                      const std::vector<float>& d) const;
};


#endif