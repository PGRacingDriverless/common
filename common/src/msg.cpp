#include "common/msg.hpp"

geometry_msgs::msg::Point create_point(
    const double x,
    const double y,
    const double z)
{
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::msg::Pose create_pose(
    const geometry_msgs::msg::Point point,
    const geometry_msgs::msg::Quaternion quaternion)
{
    geometry_msgs::msg::Pose pose;
    pose.position = point;
    pose.orientation = quaternion;
    return pose;
}

geometry_msgs::msg::Quaternion create_quaternion(
    const double x,
    const double y,
    const double z,
    const double w)
{
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = x;
    quaternion.y = y;
    quaternion.z = z;
    quaternion.w = w;
    return quaternion;
}

geometry_msgs::msg::Vector3 create_vector3(
    const double x,
    const double y,
    const double z)
{
    geometry_msgs::msg::Vector3 vector;
    vector.x = x;
    vector.y = y;
    vector.z = z;
    return vector;
}
