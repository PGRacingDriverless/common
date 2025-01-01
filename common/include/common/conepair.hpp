#ifndef COMMON_CONEPAIR_HPP
#define COMMON_CONEPAIR_HPP

#include "cone.hpp"

#include <boost/property_tree/json_parser.hpp>
using boost::property_tree::ptree;

namespace common::cones
{
    class ConePair
    {
    public:
        ConePair() = default;
        ConePair(const Cone &cone_outer, const Cone &cone_inner);

        Cone getOuter() const;
        void setOuter(const Cone &cone_outer);

        Cone getInner() const;
        void setInner(const Cone &cone_inner);

        bool operator==(const ConePair &cone_pair) const;
        bool operator!=(const ConePair &cone_pair) const;

        void read_from_json(const ptree::value_type &item);
        static ConePair get_from_json(const ptree::value_type &item);

    private:
        Cone cone_outer;
        Cone cone_inner;
    };
};

#endif