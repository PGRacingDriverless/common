#ifndef COMMON_MSG_HPP
#define COMMON_MSG_HPP

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

/**
 * Creates `geometry_msgs` `Point` with given `x`,
 * `y` and `z`.
 * @param x double x-axis value.
 * @param y double y-axis value.
 * @param z double z-axis value.
 * @return `geometry_msgs::msg::Point` point with given `x`, `y` and
 * `z`.
 */
geometry_msgs::msg::Point create_point(
    const double x,
    const double y,
    const double z);

/**
 * Creates `geometry_msgs` `Pose` with given
 * position and orientation.
 * @param point `geometry_msgs::msg::Point` position.
 * @param quaternion `geometry_msgs::msg::Quaternion` orientation.
 * @return `geometry_msgs::msg::Pose` pose with given position and
 * orientation.
 */
geometry_msgs::msg::Pose create_pose(
    const geometry_msgs::msg::Point point,
    const geometry_msgs::msg::Quaternion quaternion);

/**
 * Creates `geometry_msgs` `Quaternion` with
 * given `x`, `y`, `z` and `w`.
 * @param x double x of vector of imaginary part.
 * @param y double y of vector of imaginary part.
 * @param z double z of vector of imaginary part.
 * @param w double w scalar of real part.
 * @return `geometry_msgs::msg::Quaternion` quaternion with given
 * `x`, `y`, `z` and `w`.
 */
geometry_msgs::msg::Quaternion create_quaternion(
    const double x,
    const double y,
    const double z,
    const double w);

/**
 * Creates `geometry_msgs` `Vector3` with given
 * `x`, `y` and `z`.
 * @param x double x value.
 * @param y double y value.
 * @param z double z value.
 * @return `geometry_msgs::msg::Vector3` vector with given `x`, `y`
 * and `z`.
 */
geometry_msgs::msg::Vector3 create_vector3(
    const double x,
    const double y,
    const double z);


#endif
