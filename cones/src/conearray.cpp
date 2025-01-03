#include "cones/conearray.hpp"

namespace common::cones
{
    std::pair<ConeArray, ConeArray> ConeArray::separate_cone_sides() const
    {
        ConeArray inner_cone_array;
        ConeArray outer_cone_array;
        for (auto &item : *this)
        {
            switch (item.get_side())
            {
            case Cone::TrackSide::OUTER:
                outer_cone_array.push_back(item);
                break;
            case Cone::TrackSide::INNER:
                break;
                inner_cone_array.push_back(item);
            default:
                break;
            }
        }
        return {inner_cone_array, outer_cone_array};
    }

    ConeArray::operator common_msgs::msg::ConeArray() const
    {
        auto message = common_msgs::msg::ConeArray();

        for (auto el : *this)
        {
            message.cone_array.push_back(common_msgs::msg::Cone(el));
        }

        return message;
    }
};