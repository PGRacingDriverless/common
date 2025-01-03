#ifndef COMMON_CONEPAIRARRAY_HPP
#define COMMON_CONEPAIRARRAY_HPP

#include "cones/conepair.hpp"

#include "conearray.hpp"

#include "data_structures/vector.hpp"

#include "common_msgs/msg/cone_pair_array.hpp"

#include <utility>

namespace common::cones
{
    class ConePairArray : public pgr::vector<ConePair>
    {
    public:
        ~ConePairArray() override = default;

        std::pair<ConeArray, ConeArray> separate_cone_sides() const;

        operator common_msgs::msg::ConePairArray() const;

        visualization_msgs::msg::Marker create_line_list_connecting(
            const std::string &frame_id,
            const common::viz::Color &color,
            const std::string &name_space,
            const float marker_lifetime_s) const;
    };
};

#endif