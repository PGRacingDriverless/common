#ifndef COMMON_CONES_HPP
#define COMMON_CONES_HPP

#include <vector>

#include <lemon/list_graph.h>
#include <lemon/path.h>

#include "cones/cone.hpp"
#include "cones/conepair.hpp"
#include "cones/conegraph.hpp"
#include "cones/conearray.hpp"

namespace common::cones
{
    using ConeArray = std::vector<Cone>;
    using ConePairArray = std::vector<ConePair>;

    inline void separate_cone_sides(const ConeArray &input_cone_array, common::cones::ConeArray &inner_cone_array, common::cones::ConeArray &outer_cone_array);

    inline void separate_cone_sides_from_cone_pairs(const common::cones::ConePairArray &cone_pair_array, common::cones::ConeArray &inner_cone_array, common::cones::ConeArray &outer_cone_array);
};

#endif