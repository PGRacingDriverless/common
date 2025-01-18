#ifndef COMMON_CONE_HPP
#define COMMON_CONE_HPP

#include <pcl/point_types.h>

#include <string>
#include "common_msgs/msg/cone_array.hpp"

enum Color {
    CBLUE,
    CYELLOW,
    CORANGE
};

enum Side {
    LEFT,
    RIGHT
};

struct Cone {
    Color color;
    Side side;
    pcl::PointXY position;
};

struct ConePair {
    Cone left;
    Cone right;
};
//  

std::pair<std::vector<Cone>, std::vector<Cone>> separate_cone_sides(const std::vector<Cone> &cone_array);
std::pair<std::vector<Cone>, std::vector<Cone>> separate_cone_sides_msg(const common_msgs::msg::ConeArray::SharedPtr cone_array_msg);
std::pair<std::vector<Cone>, std::vector<Cone>> separate_cone_pair_sides(const std::vector<ConePair> &cone_pair_array);

#endif