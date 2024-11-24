#include "common/cones.hpp"

void separate_cone_sides(const std::vector<Cone>& input_cone_array, ConeArray& inner_cone_array, ConeArray& outer_cone_array){
    for(auto& item: input_cone_array){
        if(item.get_side() == Cone::TrackSide::OUTER){
            outer_cone_array.add_cone(item);
        } 
        else if(item.get_side() == Cone::TrackSide::INNER) {
            inner_cone_array.add_cone(item);
        }
    }
}

void separate_cone_sides_from_cone_pairs(ConePairArray &cone_pair_array, ConeArray& inner_cone_array, ConeArray& outer_cone_array){
    for(auto& item: cone_pair_array.get_pairs()){
        inner_cone_array.add_cone(item.cone_inner);
        outer_cone_array.add_cone(item.cone_outer);
    }
}

bool check_pairs_equality(const ConePair &pair1, const ConePair &pair2)
{
    bool inner_cone_equality = false;
    bool outer_cone_equality = false;

    if((pair1.cone_inner.get_x() == pair2.cone_inner.get_x()) && (pair1.cone_inner.get_y() == pair2.cone_inner.get_y())){
        inner_cone_equality = true;
    }

    if((pair1.cone_outer.get_x() == pair2.cone_outer.get_x()) && (pair1.cone_outer.get_y() == pair2.cone_outer.get_y())){
        outer_cone_equality = true;
    }

    return inner_cone_equality && outer_cone_equality;
}