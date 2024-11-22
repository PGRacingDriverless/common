#ifndef COMMON_HPP
#define COMMON_HPP

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace common {
namespace msgs {

/**
 * `inline` function. Creates `geometry_msgs` `Point` with given `x`,
 * `y` and `z`.
 * @param x double x-axis value.
 * @param y double y-axis value.
 * @param z double z-axis value.
 * @return `geometry_msgs::msg::Point` point with given `x`, `y` and
 * `z`.
 */
inline geometry_msgs::msg::Point create_point(
    const double x, const double y, const double z
) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

/**
 * `inline` function. Creates `geometry_msgs` `Pose` with given
 * position and orientation.
 * @param point `geometry_msgs::msg::Point` position.
 * @param quaternion `geometry_msgs::msg::Quaternion` orientation.
 * @return `geometry_msgs::msg::Pose` pose with given position and
 * orientation.
 */
inline geometry_msgs::msg::Pose create_pose(
    const geometry_msgs::msg::Point point,
    const geometry_msgs::msg::Quaternion quaternion
) {
    geometry_msgs::msg::Pose pose;
    pose.position = point;
    pose.orientation = quaternion;
    return pose;
}

/**
 * `inline` function. Creates `geometry_msgs` `Quaternion` with
 * given `x`, `y`, `z` and `w`.
 * @param x double x of vector of imaginary part.
 * @param y double y of vector of imaginary part.
 * @param z double z of vector of imaginary part.
 * @param w double w scalar of real part.
 * @return `geometry_msgs::msg::Quaternion` quaternion with given
 * `x`, `y`, `z` and `w`.
 */
inline geometry_msgs::msg::Quaternion create_quaternion(
    const double x, const double y, const double z, const double w
) {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = x;
    quaternion.y = y;
    quaternion.z = z;
    quaternion.w = w;
    return quaternion;
}

/**
 * `inline` function. Creates `geometry_msgs` `Vector3` with given
 * `x`, `y` and `z`.
 * @param x double x value.
 * @param y double y value.
 * @param z double z value.
 * @return `geometry_msgs::msg::Vector3` vector with given `x`, `y`
 * and `z`.
 */
inline geometry_msgs::msg::Vector3 create_vector3(
    const double x, const double y, const double z
) {
  geometry_msgs::msg::Vector3 vector;
  vector.x = x;
  vector.y = y;
  vector.z = z;
  return vector;
}

}
}

#endif
