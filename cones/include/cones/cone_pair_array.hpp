#ifndef COMMON_CONEPAIRARRAY_HPP
#define COMMON_CONEPAIRARRAY_HPP

#include "cones/cone_pair.hpp"
#include "cone_array.hpp"
#include "data_structures/vector.hpp"
#include "common_msgs/msg/cone_pair_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <utility>

class ConePairArray : public vector<ConePair>
{
public:
    ConePairArray() = default;
    ConePairArray(const common_msgs::msg::ConePairArray::SharedPtr msg);
    ConePairArray(const ConeArray &left, const ConeArray &right);

    ~ConePairArray() override = default;

    std::pair<ConeArray, ConeArray> separate_cone_sides() const;

    explicit operator common_msgs::msg::ConePairArray() const;

    visualization_msgs::msg::Marker create_line_list_connecting(
        const std::string &frame_id,
        const color_t &color,
        const std::string &name_space,
        const float marker_lifetime_s) const;
};

#endif