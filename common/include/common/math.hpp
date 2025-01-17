#ifndef COMMON_MATH_HPP
#define COMMON_MATH_HPP

#include <pcl/point_types.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>

double dist(const double x1, const double y1, const double x2, const double y2);

pcl::PointXY calculate_direction_vector(const pcl::PointXY &p1, const pcl::PointXY &p2);
pcl::PointXY calculate_direction_vector(const geometry_msgs::msg::Pose &pose);

float quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q);

#endif