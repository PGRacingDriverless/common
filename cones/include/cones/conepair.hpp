#ifndef COMMON_CONEPAIR_HPP
#define COMMON_CONEPAIR_HPP

#include "cone.hpp"

#include <boost/property_tree/json_parser.hpp>
using boost::property_tree::ptree;

#include "common_msgs/msg/cone_pair_array.hpp"

namespace common::cones
{
    class ConePair
    {
    public:
        ConePair() = default;
        ConePair(const Cone &cone_left, const Cone &cone_right);

        Cone getLeft() const;
        void setLeft(const Cone &cone_left);

        Cone getRight() const;
        void setRight(const Cone &cone_right);

        bool operator==(const ConePair &cone_pair) const;
        bool operator!=(const ConePair &cone_pair) const;

        void read_from_json(const ptree::value_type &item);
        static ConePair get_from_json(const ptree::value_type &item);

        explicit operator common_msgs::msg::ConePair() const;

    private:
        Cone cone_left;
        Cone cone_right;
    };
};

#endif