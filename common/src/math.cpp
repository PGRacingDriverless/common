#include "common/math.hpp"

double dist(const double x1, const double y1, const double x2, const double y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

pcl::PointXY calculate_direction_vector(
    const pcl::PointXY &p1,
    const pcl::PointXY &p2
) {
    pcl::PointXY direction;
    direction.x = p2.x - p1.x;
    direction.y = p2.y - p1.y;
    return direction;
}

pcl::PointXY calculate_direction_vector(
    const geometry_msgs::msg::Pose &pose
) {
    float yaw = quaternion_to_yaw(pose.orientation);

    pcl::PointXY direction;
    direction.x = std::cos(yaw);
    direction.y = std::sin(yaw);

    return direction;
}

float quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q)
{
    // Convert quaternion to yaw (rotation around the Z axis)
    float siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}