#include "cones/conepairarray.hpp"

namespace common::cones
{
    std::pair<ConeArray, ConeArray> ConePairArray::separate_cone_sides() const
    {
        common::cones::ConeArray inner_cone_array;
        common::cones::ConeArray outer_cone_array;
        for (auto &item : *this)
        {
            inner_cone_array.push_back(item.getInner());
            outer_cone_array.push_back(item.getOuter());
        }
        return {inner_cone_array, outer_cone_array};
    }

    ConePairArray::operator common_msgs::msg::ConePairArray() const
    {
        auto message = common_msgs::msg::ConePairArray();
        for (auto el : *this)
        {
            message.cone_pair_array.push_back(common_msgs::msg::ConePair(el));
        }

        return message;
    }

    // Takes ConePairArray and connects all points beetween cone pairs
    visualization_msgs::msg::Marker ConePairArray::create_line_list_connecting(
        const std::string &frame_id,
        const common::viz::color_t &color,
        const std::string &name_space,
        const float marker_lifetime_s) const
    {
        std::size_t marker_id = 0;
        visualization_msgs::msg::Marker cone_marker_list;

        common::viz::set_marker_parameters(
            cone_marker_list,
            color,
            name_space,
            frame_id,
            marker_lifetime_s,
            marker_id,
            visualization_msgs::msg::Marker::LINE_LIST,
            visualization_msgs::msg::Marker::ADD,
            0.05, 0.05, 0.05);

        for (std::size_t i = 0; i < size(); i++)
        {
            cone_marker_list.points.push_back(geometry_msgs::msg::Point((*this)[i].getInner()));
            cone_marker_list.points.push_back(geometry_msgs::msg::Point((*this)[i].getOuter()));
        }

        marker_id++;
        return cone_marker_list;
    }
};