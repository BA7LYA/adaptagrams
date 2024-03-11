///
/// @file PointPair.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __POINT_PAIR_HXX_3254BB97DA9C__
#define __POINT_PAIR_HXX_3254BB97DA9C__

#include <set>

namespace avoid {

class PointPair
{
public:
    PointPair(const Point& centerPoint, VertInf* inf)
        : vInf(inf)
        , centerPoint(centerPoint)
    {
        angle    = rotationalAngle(vInf->point - centerPoint);
        distance = euclideanDist(centerPoint, vInf->point);
    }

    bool operator<(const PointPair& rhs) const
    {
        // Firstly order by angle.
        if (angle == rhs.angle)
        {
            // If the points are collinear, then order them in increasing
            // distance from the point we are sweeping around.
            if (distance == rhs.distance)
            {
                // XXX: Add this assertion back if we require that
                //      connector endpoints have unique IDs. For the
                //      moment it is okay for them to have the same ID.
                // COLA_ASSERT(vInf->id != rhs.vInf->id);

                // If comparing two points at the same physical
                // position, then order them by their VertIDs.
                return vInf->id < rhs.vInf->id;
            }
            return distance < rhs.distance;
        }
        return angle < rhs.angle;
    }

    VertInf* vInf;
    double   angle;
    double   distance;
    Point    centerPoint;
};

using VertSet = std::set<PointPair>;

}  // namespace avoid

#endif  // __POINT_PAIR_HXX_3254BB97DA9C__
