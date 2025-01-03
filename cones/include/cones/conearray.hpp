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

        visualization_msgs::msg::Marker create_cube_list(
            const std::string &frame_id,
            const common::viz::Color &color,
            const std::string &name_space,
            const float marker_lifetime_s) const;

        visualization_msgs::msg::Marker create_line_list(
            const std::string &frame_id,
            const common::viz::Color &color,
            const std::string &name_space,
            const float marker_lifetime_s) const;

        visualization_msgs::msg::MarkerArray create_id_labels(
            const std::string frame_id,
            const std::string text,
            const std::string name_space,
            const float marker_lifetime_s) const;
    };
};

#endif