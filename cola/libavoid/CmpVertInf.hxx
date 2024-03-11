///
/// @file CmpVertInf.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __CMP_VERT_INF_HXX_47CAD4125984__
#define __CMP_VERT_INF_HXX_47CAD4125984__

#include <set>

namespace avoid {

struct CmpVertInf
{
    bool operator()(const VertInf* u, const VertInf* v) const
    {
        // Comparator for VertSet, an ordered set of VertInf pointers.
        // It is assumed vertical sets of points will all have the same
        // x position and horizontal sets all share a y position, so this
        // method can be used to sort both these sets.
        COLA_ASSERT((u->point.x == v->point.x) || (u->point.y == v->point.y));
        if (u->point.x != v->point.x)
        {
            return u->point.x < v->point.x;
        }
        else if (u->point.y != v->point.y)
        {
            return u->point.y < v->point.y;
        }
        return u < v;
    }
};

using VertSet = std::set<VertInf*, CmpVertInf>;

}  // namespace avoid

#endif  // __CMP_VERT_INF_HXX_47CAD4125984__
