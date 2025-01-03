#include "cones/conepairarray.hpp"

namespace common::cones
{
    std::pair<ConeArray, ConeArray> ConePairArray::separate_cone_sides() const
    {
        common::cones::ConeArray inner_cone_array;
        common::cones::ConeArray outer_cone_array;
        for (auto &item : *this)
        {
            inner_cone_array.push_back(item.getInner());
            outer_cone_array.push_back(item.getOuter());
        }
        return {inner_cone_array, outer_cone_array};
    }

    ConePairArray::operator common_msgs::msg::ConePairArray() const
    {
        auto message = common_msgs::msg::ConePairArray();
        for (auto el: *this)
        {
            message.cone_pair_array.push_back(common_msgs::msg::ConePair(el));
        }

        return message;
    }
};