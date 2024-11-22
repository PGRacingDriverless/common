#ifndef COMMON_CONE_HPP
#define COMMON_CONE_HPP

#include "cones/cone_array.hpp"
#include "cones/cone_pair_array.hpp"

bool check_pairs_equality(const ConePair &pair1, const ConePair &pair2);
void separate_cone_sides(const std::vector<Cone>& input_cone_array, ConeArray& inner_cone_array, ConeArray& outer_cone_array);
void separate_cone_sides_from_cone_pairs(ConePairArray &cone_pair_array, ConeArray& inner_cone_array, ConeArray& outer_cone_array);

#endif