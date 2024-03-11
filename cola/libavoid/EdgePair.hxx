///
/// @file EdgePair.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __EDGE_PAIR_HXX_6B7F226CA80B__
#define __EDGE_PAIR_HXX_6B7F226CA80B__

#include <list>

namespace avoid {

class EdgePair
{
public:
    EdgePair()
        : vInf1(nullptr)
        , vInf2(nullptr)
        , dist1(0.0)
        , dist2(0.0)
        , angle(0.0)
        , angleDist(0.0)
    {
        // The default constuctor should never be called.
        // This is defined to appease the MSVC compiler.
        COLA_ASSERT(false);
    }

    EdgePair(const PointPair& p1, VertInf* v)
        : vInf1(p1.vInf)
        , vInf2(v)
        , dist1(p1.distance)
        , dist2(euclideanDist(vInf2->point, p1.centerPoint))
        , angle(p1.angle)
        , angleDist(p1.distance)
        , centerPoint(p1.centerPoint)
    {
    }

    bool operator<(const EdgePair& rhs) const
    {
        COLA_ASSERT(angle == rhs.angle);
        if (angleDist == rhs.angleDist)
        {
            return (dist2 < rhs.dist2);
        }
        return (angleDist < rhs.angleDist);
    }

    bool operator==(const EdgePair& rhs) const
    {
        if (((vInf1->id == rhs.vInf1->id) && (vInf2->id == rhs.vInf2->id))
            || ((vInf1->id == rhs.vInf2->id) && (vInf2->id == rhs.vInf1->id)))
        {
            return true;
        }
        return false;
    }

    bool operator!=(const EdgePair& rhs) const
    {
        if (((vInf1->id == rhs.vInf1->id) && (vInf2->id == rhs.vInf2->id))
            || ((vInf1->id == rhs.vInf2->id) && (vInf2->id == rhs.vInf1->id)))
        {
            return false;
        }
        return true;
    }

    void setNegativeAngle(void)
    {
        angle = -1.0;
    }

    double setCurrAngle(const PointPair& p)
    {
        if (p.vInf->point == vInf1->point)
        {
            angleDist = dist1;
            angle     = p.angle;
        }
        else if (p.vInf->point == vInf2->point)
        {
            angleDist = dist2;
            angle     = p.angle;
        }
        else if (p.angle != angle)
        {
            COLA_ASSERT(p.angle > angle);
            angle = p.angle;
            Point pp;
            int   result = rayIntersectPoint(
                vInf1->point,
                vInf2->point,
                centerPoint,
                p.vInf->point,
                &(pp.x),
                &(pp.y)
            );
            if (result != DO_INTERSECT)
            {
                // This can happen with points that appear to have the
                // same angle but at are at slightly different positions
                angleDist = std::min(dist1, dist2);
            }
            else
            {
                angleDist = euclideanDist(pp, centerPoint);
            }
        }

        return angleDist;
    }

    VertInf* vInf1;
    VertInf* vInf2;
    double   dist1;
    double   dist2;
    double   angle;
    double   angleDist;
    Point    centerPoint;
};

using SweepEdgeList = std::list<EdgePair>;

}  // namespace avoid

#endif  // __EDGE_PAIR_HXX_6B7F226CA80B__
