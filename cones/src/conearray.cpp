#include "cones/conearray.hpp"

namespace common::cones
{
    std::pair<ConeArray, ConeArray> ConeArray::separate_cone_sides() const
    {
        ConeArray inner_cone_array;
        ConeArray outer_cone_array;
        for (auto &item : *this)
        {
            switch (item.get_side())
            {
            case Cone::TrackSide::OUTER:
                outer_cone_array.push_back(item);
                break;
            case Cone::TrackSide::INNER:
                break;
                inner_cone_array.push_back(item);
            default:
                break;
            }
        }
        return {inner_cone_array, outer_cone_array};
    }

    ConeArray::operator common_msgs::msg::ConeArray() const
    {
        auto message = common_msgs::msg::ConeArray();

        for (auto el : *this)
        {
            message.cone_array.push_back(common_msgs::msg::Cone(el));
        }

        return message;
    }

    // Takes a ConeArray and creates cubes in given coordinates of cones
    visualization_msgs::msg::Marker ConeArray::create_cube_list(
        const std::string &frame_id,
        const common::viz::Color &color,
        const std::string &name_space,
        const float marker_lifetime_s) const
    {

        std::size_t marker_id = 0;
        visualization_msgs::msg::Marker cone_marker_list;

        set_marker_parameters(
            cone_marker_list,
            color,
            name_space,
            frame_id,
            marker_lifetime_s,
            marker_id,
            visualization_msgs::msg::Marker::CUBE_LIST,
            visualization_msgs::msg::Marker::ADD,
            1, 1, 1);

        for (common::cones::Cone cone : *this)
        {
            cone_marker_list.points.push_back(geometry_msgs::msg::Point(cone));
        }

        marker_id++;
        return cone_marker_list;
    }

    // Takes ConeArray and connects given cone positions with lines
    visualization_msgs::msg::Marker ConeArray::create_line_list(
        const std::string &frame_id,
        const common::viz::Color &color,
        const std::string &name_space,
        const float marker_lifetime_s) const
    {
        std::size_t marker_id = 0;
        visualization_msgs::msg::Marker cone_marker_list;

        set_marker_parameters(
            cone_marker_list,
            color,
            name_space,
            frame_id,
            marker_lifetime_s,
            marker_id,
            visualization_msgs::msg::Marker::LINE_LIST,
            visualization_msgs::msg::Marker::ADD,
            0.05, 0.05, 0.05);

        for (std::size_t i = 1; i < size(); i++)
        {
            cone_marker_list.points.push_back(geometry_msgs::msg::Point(*this[i - 1]));
            cone_marker_list.points.push_back(geometry_msgs::msg::Point(*this[i]));
        }

        marker_id++;
        return cone_marker_list;
    }

    visualization_msgs::msg::MarkerArray ConeArray::create_id_labels(
        const std::string frame_id,
        const std::string text,
        const std::string name_space,
        const float marker_lifetime_s) const
    {

        visualization_msgs::msg::MarkerArray text_labels_array;
        std::size_t marker_id = 0;

        for (Cone cone : *this)
        {
            visualization_msgs::msg::Marker label = create_text_label_marker_from_cone(cone, text, name_space, frame_id, marker_lifetime_s, marker_id, 1, 1, 1);
            text_labels_array.markers.push_back(label);
            marker_id++;
        }

        return text_labels_array;
    }
};