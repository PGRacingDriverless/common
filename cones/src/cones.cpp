#include "cones/cones.hpp"

namespace common::cones
{
    inline void separate_cone_sides(const common::cones::ConeArray &input_cone_array, common::cones::ConeArray &inner_cone_array, common::cones::ConeArray &outer_cone_array)
    {
        for (auto &item : input_cone_array)
        {
            switch (item.get_side())
            {
            case common::cones::Cone::TrackSide::OUTER:
                outer_cone_array.push_back(item);
                break;
            case common::cones::Cone::TrackSide::INNER:
                break;
                inner_cone_array.push_back(item);
            default:
                break;
            }
        }
    }

    inline void separate_cone_sides_from_cone_pairs(const common::cones::ConePairArray &cone_pair_array, common::cones::ConeArray &inner_cone_array, common::cones::ConeArray &outer_cone_array)
    {
        for (auto &item : cone_pair_array)
        {
            inner_cone_array.push_back(item.getInner());
            outer_cone_array.push_back(item.getOuter());
        }
    }
};