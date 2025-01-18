#ifndef COMMON_CONE_HPP
#define COMMON_CONE_HPP

#include <pcl/point_types.h>

#include <string>

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

// ConePair get_cone_pair_from_json(const ptree::value_type &item);
std::pair<std::vector<Cone>, std::vector<Cone>> separate_cone_sides(const std::vector<Cone> &cone_array);
std::pair<std::vector<Cone>, std::vector<Cone>> separate_cone_pair_sides(const std::vector<ConePair> &cone_pair_array);

#endif