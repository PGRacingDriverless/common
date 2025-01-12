#include "cones/cone_pair.hpp"

ConePair::ConePair(const Cone &cone_left, const Cone &cone_right) : cone_left(cone_left), cone_right(cone_right) {}

Cone ConePair::getLeft() const
{
    return cone_left;
}
void ConePair::setLeft(const Cone &cone_left)
{
    this->cone_left = cone_left;
}

Cone ConePair::getRight() const
{
    return cone_right;
}
void ConePair::setRight(const Cone &cone_right)
{
    this->cone_right = cone_right;
}

bool ConePair::operator==(const ConePair &cone_pair) const
{
    return cone_pair.cone_left == cone_left && cone_pair.cone_right == cone_right;
}
bool ConePair::operator!=(const ConePair &cone_pair) const
{
    return !(*this == cone_pair);
}

void ConePair::read_from_json(const ptree::value_type &item)
{
    cone_left.set_x(item.second.get<double>("cone1.x"));
    cone_left.set_y(item.second.get<double>("cone1.y"));
    cone_left.set_side(static_cast<Cone::TrackSide>(item.second.get<int>("cone1.side")));
    cone_left.set_color(static_cast<Cone::Color>(item.second.get<int>("cone1.color")));

    cone_right.set_x(item.second.get<double>("cone2.x"));
    cone_right.set_y(item.second.get<double>("cone2.y"));
    cone_right.set_side(static_cast<Cone::TrackSide>(item.second.get<int>("cone2.side")));
    cone_right.set_color(static_cast<Cone::Color>(item.second.get<int>("cone2.color")));
}
ConePair ConePair::get_from_json(const ptree::value_type &item)
{
    ConePair cone_pair;
    cone_pair.read_from_json(item);
    return cone_pair;
}

ConePair::operator common_msgs::msg::ConePair() const
{
    common_msgs::msg::ConePair cone_pair;

    cone_pair.cone_left = common_msgs::msg::Cone(cone_left);
    cone_pair.cone_right = common_msgs::msg::Cone(cone_right);

    return cone_pair;
}