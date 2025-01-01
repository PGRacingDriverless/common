#include "common/conepair.hpp"

namespace common::cones
{
    ConePair::ConePair(const Cone &cone_outer, const Cone &cone_inner) : cone_outer(cone_outer), cone_inner(cone_inner) {}

    Cone ConePair::getOuter() const
    {
        return cone_outer;
    }
    void ConePair::setOuter(const Cone &cone_outer)
    {
        this->cone_outer = cone_outer;
    }

    Cone ConePair::getInner() const
    {
        return cone_inner;
    }
    void ConePair::setInner(const Cone &cone_inner)
    {
        this->cone_inner = cone_inner;
    }

    bool ConePair::operator==(const ConePair &cone_pair) const
    {
        return cone_pair.cone_inner == cone_inner && cone_pair.cone_outer == cone_outer;
    }
    bool ConePair::operator!=(const ConePair &cone_pair) const
    {
        return !(*this == cone_pair);
    }

    void ConePair::read_from_json(const ptree::value_type &item)
    {
        cone_outer.set_x(item.second.get<double>("cone1.x"));
        cone_outer.set_y(item.second.get<double>("cone1.y"));
        cone_outer.set_side(static_cast<common::cones::Cone::TrackSide>(item.second.get<int>("cone1.side")));
        cone_outer.set_color(static_cast<common::cones::Cone::Color>(item.second.get<int>("cone1.color")));

        cone_inner.set_x(item.second.get<double>("cone2.x"));
        cone_inner.set_y(item.second.get<double>("cone2.y"));
        cone_inner.set_side(static_cast<common::cones::Cone::TrackSide>(item.second.get<int>("cone2.side")));
        cone_inner.set_color(static_cast<common::cones::Cone::Color>(item.second.get<int>("cone2.color")));
    }
    ConePair ConePair::get_from_json(const ptree::value_type &item)
    {
        common::cones::ConePair cone_pair;
        cone_pair.read_from_json(item);
        return cone_pair;
    }
};