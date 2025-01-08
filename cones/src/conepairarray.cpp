#include "cones/conepairarray.hpp"

namespace common::cones
{
    ConePairArray::ConePairArray(const common_msgs::msg::ConePairArray::SharedPtr msg)
    {
        size_t num_cone_pairs = msg->cone_pair_array.size();
        if (0 == num_cone_pairs)
        {
            return;
        }
        for (const auto &msg_cone_pair : msg->cone_pair_array)
        {
            common::cones::ConePair cone_pair;

            cone_pair.setLeft(common::cones::Cone(msg_cone_pair.cone_left.x, msg_cone_pair.cone_right.y));
            cone_pair.setRight(common::cones::Cone(msg_cone_pair.cone_left.x, msg_cone_pair.cone_right.y));

            // Log the cone pair
            // RCLCPP_INFO(this->get_logger(), "Received cone pair: cone1(%f, %f), cone2(%f, %f)",
            //            cone_pair.cone_left.get_x(), cone_pair.cone_left.get_y(), cone_pair.cone_right.get_x(), cone_pair.cone_right.get_y());

            (*this).push_back(cone_pair);
        }
    }

    ConePairArray::ConePairArray(const ConeArray &left, const ConeArray &right)
    {
        for (size_t i = 0; i < left.size() && i < right.size(); i++)
        {
            common::cones::ConePair pair;
            pair.setLeft(left[i]);
            pair.setRight(right[i]);
            push_back(pair);
        }
    }

    std::pair<ConeArray, ConeArray> ConePairArray::separate_cone_sides() const
    {
        common::cones::ConeArray left_cone_array;
        common::cones::ConeArray right_cone_array;
        for (auto &item : *this)
        {
            left_cone_array.push_back(item.getLeft());
            right_cone_array.push_back(item.getRight());
        }
        return {left_cone_array, right_cone_array};
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
            cone_marker_list.points.push_back(geometry_msgs::msg::Point((*this)[i].getLeft()));
            cone_marker_list.points.push_back(geometry_msgs::msg::Point((*this)[i].getRight()));
        }

        marker_id++;
        return cone_marker_list;
    }
};