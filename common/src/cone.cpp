#include "common/cone.hpp"

ConePair get_cone_pair_from_json(const ptree::value_type &item)
{
    ConePair pair;
    pair.left.position.x = item.second.get<double>("cone1.x");
    pair.left.position.y = item.second.get<double>("cone1.y");
    pair.left.side = static_cast<Side>(item.second.get<int>("cone1.side"));
    pair.left.color = static_cast<Color>(item.second.get<int>("cone1.color"));

    pair.right.position.x = item.second.get<double>("cone2.x");
    pair.right.position.y = item.second.get<double>("cone2.y");
    pair.right.side = static_cast<Side>(item.second.get<int>("cone2.side"));
    pair.right.color = static_cast<Color>(item.second.get<int>("cone2.color"));

    return pair;
}

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