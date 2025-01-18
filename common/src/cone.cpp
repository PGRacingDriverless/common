#include "common/cone.hpp"

std::pair<std::vector<Cone>, std::vector<Cone>> separate_cone_sides(const std::vector<Cone> &cone_array)
{
    std::vector<Cone> left_cone_array;
    std::vector<Cone> right_cone_array;
    for (const auto &cone : cone_array) {
        Cone new_cone;
        new_cone.position.x = cone.position.x;
        new_cone.position.y = cone.position.y;

        if (cone.side == 0) {
            new_cone.side = Side::LEFT;
            new_cone.color = Color::CBLUE;
            left_cone_array.push_back(new_cone);
        } else {
            new_cone.side = Side::RIGHT;
            new_cone.color = Color::CYELLOW;
            right_cone_array.push_back(new_cone);
        }
    }
    return {left_cone_array, right_cone_array};
}

std::pair<std::vector<Cone>, std::vector<Cone>> separate_cone_sides_msg(const common_msgs::msg::ConeArray::SharedPtr cone_array_msg)
{
    std::vector<Cone> left_cone_array;
    std::vector<Cone> right_cone_array;
    for (const auto &cone : cone_array_msg->cone_array) {
        Cone new_cone;
        new_cone.position.x = cone.x;
        new_cone.position.y = cone.y;

        if (cone.side == 0) {
            new_cone.side = Side::LEFT;
            new_cone.color = Color::CBLUE;
            left_cone_array.push_back(new_cone);
        } else {
            new_cone.side = Side::RIGHT;
            new_cone.color = Color::CYELLOW;
            right_cone_array.push_back(new_cone);
        }
    }
    return {left_cone_array, right_cone_array};
}

std::pair<std::vector<Cone>, std::vector<Cone>> separate_cone_pair_sides(const std::vector<ConePair> &cone_pair_array)
{
    std::vector<Cone> left_cone_array;
    std::vector<Cone> right_cone_array;
    for (auto &item : cone_pair_array)
    {
        left_cone_array.push_back(item.left);
        right_cone_array.push_back(item.right);
    }
    return {left_cone_array, right_cone_array};
}
