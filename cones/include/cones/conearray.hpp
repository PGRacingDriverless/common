#ifndef COMMON_CONEARRAY_HPP
#define COMMON_CONEARRAY_HPP

#include "cones/cone.hpp"

#include "data_structures/vector.hpp"

#include <utility>

namespace common::cones
{
    class ConeArray : public pgr::vector<Cone>
    {
    public:
        ~ConeArray() override = default;

        std::pair<ConeArray, ConeArray> separate_cone_sides() const;

        operator common_msgs::msg::ConeArray() const;
    };
};

#endif