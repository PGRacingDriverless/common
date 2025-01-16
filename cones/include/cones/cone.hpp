#ifndef COMMON_CONE_HPP
#define COMMON_CONE_HPP

#include "visualization_msgs/msg/marker.hpp"
#include "common_msgs/msg/cone_array.hpp"
#include "common/visualization.hpp"

#include <string>

//
#include <pcl/point_types.h>

enum ColorNew {
    CBLUE,
    CYELLOW,
    CORANGE
};

enum SideNew {
    LEFT,
    RIGHT
};

struct ConeNew {
    ColorNew color;
    SideNew side;
    pcl::PointXY position;
};

struct ConePairNew {
    ConeNew left;
    ConeNew right;
};
//

class Cone
{
public:
    enum TrackSide
    {
        LEFT,
        RIGHT
    };
    enum Color
    {
        BLUE,
        YELLOW,
        ORANGE
    };

    Cone() = default;
    Cone(double x, double y);
    Cone(double x, double y, TrackSide side, Color color);
    Cone(const Cone &cone);

    double get_x() const;
    void set_x(double x);

    double get_y() const;
    void set_y(double y);

    void setPos(double x, double y);

    TrackSide get_side() const;
    void set_side(TrackSide side);

    Color get_color() const;
    void set_color(Color color);

    bool operator==(const Cone &cone) const;
    bool operator!=(const Cone &cone) const;

    explicit operator geometry_msgs::msg::Point() const;
    explicit operator common_msgs::msg::Cone() const;

    visualization_msgs::msg::Marker create_rviz_visualization_message(const std::string &name_space, const int marker_count) const;
    visualization_msgs::msg::Marker create_text_label_marker(
        const std::string &text,
        const std::string &name_space,
        const std::string &frame_id,
        const float marker_lifetime_s,
        const size_t marker_id,
        const float scale_x,
        const float scale_y,
        const float scale_z) const;

    static visualization_msgs::msg::Marker create_rviz_line_visualization_message(const std::string &name_space, const Cone &cone1, const Cone &cone2, const size_t marker_id);

private:
    double m_x = 0;
    double m_y = 0;
    TrackSide m_side = LEFT;
    Color m_color = YELLOW;
};

#endif