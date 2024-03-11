/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2004-2014  Monash University
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * See the file LICENSE.LGPL distributed with the library.
 *
 * Licensees holding a valid commercial license may use this file in
 * accordance with the commercial license agreement provided with the
 * library.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author(s):  Michael Wybrow
 */

// For M_PI.
// This should be first include for MSVC.
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "libavoid/AStarPath.hxx"

#include <algorithm>
#include <cfloat>
#include <climits>
#include <cmath>
#include <vector>

#include "libavoid/assertions.h"
#include "libavoid/connector.h"
#include "libavoid/debug.h"
#include "libavoid/debughandler.h"
#include "libavoid/geometry.h"
#include "libavoid/graph.h"
#include "libavoid/router.h"
#include "libavoid/vertices.h"
#include "libavoid/viscluster.h"

// #define ESTIMATED_COST_DEBUG

namespace avoid {

static double Dot(const Point& l, const Point& r)
{
    return (l.x * r.x) + (l.y * r.y);
}

static double CrossLength(const Point& l, const Point& r)
{
    return (l.x * r.y) - (l.y * r.x);
}

// Return the angle between the two line segments made by the
// points p1--p2 and p2--p3.  Return value is in radians.
//
static double angleBetween(const Point& p1, const Point& p2, const Point& p3)
{
    if ((p1.x == p2.x && p1.y == p2.y) || (p2.x == p3.x && p2.y == p3.y))
    {
        // If two of the points are the same, then we can't say anything
        // about the angle between.  Treat them as being collinear.
        return M_PI;
    }

    Point v1(p1.x - p2.x, p1.y - p2.y);
    Point v2(p3.x - p2.x, p3.y - p2.y);

    return fabs(atan2(CrossLength(v1, v2), Dot(v1, v2)));
}

// Construct a temporary Polygon path given several VertInf's for a connector.
//
static void constructPolygonPath(
    Polygon& connRoute,
    VertInf* inf2,
    VertInf* inf3,
    ANode*   inf1Node
)
{
    // Don't include colinear points.
    bool simplified = true;

    int routeSize = 2;
    for (ANode* curr = inf1Node; curr != nullptr; curr = curr->prevNode)
    {
        routeSize += 1;
    }
    connRoute.ps.resize(routeSize);
    int arraySize                = routeSize;
    connRoute.ps[routeSize - 1]  = inf3->point;
    connRoute.ps[routeSize - 2]  = inf2->point;
    routeSize                   -= 3;
    for (ANode* curr = inf1Node; curr != nullptr; curr = curr->prevNode)
    {
        // For connection pins, we stop and don't include the fake shape
        // center as part of this path.
        bool isConnectionPin = curr->inf->id.isConnectionPin();

        if (!simplified)
        {
            // If this is non-simplified, we don't need to do anything
            // clever and can simply add the new point.
            connRoute.ps[routeSize]  = curr->inf->point;
            routeSize               -= 1;

            if (isConnectionPin)
            {
                // Stop at the connection pin.
                break;
            }
            continue;
        }

        if ((curr == inf1Node)
            || vecDir(
                   curr->inf->point,
                   connRoute.ps[routeSize + 1],
                   connRoute.ps[routeSize + 2]
               ) != 0)
        {
            // Add new point if this is the earlier than the last segment
            // and it is not colinear with the other points.
            // Note, you can't collapse the 'last' segment with previous
            // segments, or if this just intersects another line you risk
            // penalising it once for each collapsed line segment.
            connRoute.ps[routeSize]  = curr->inf->point;
            routeSize               -= 1;
        }
        else
        {
            // The last point is inline with this one, so update it.
            connRoute.ps[routeSize + 1] = curr->inf->point;
        }

        if (isConnectionPin)
        {
            // Stop at the connection pin.
            break;
        }
    }

    // If the vector is not filled, move entries to the beginning and
    // remove the unused end of the vector.
    int diff = routeSize + 1;
    COLA_ASSERT(simplified || (diff == 0));
    if (diff > 0)
    {
        for (int i = diff; i < arraySize; ++i)
        {
            connRoute.ps[i - diff] = connRoute.ps[i];
        }
        connRoute.ps.resize(connRoute.size() - diff);
    }
}

// Used to get an indication of if a diffence is positive (1),
// negative (-1) or no different (0).
static inline int dimDirection(double difference)
{
    if (difference > 0)
    {
        return 1;
    }
    else if (difference < 0)
    {
        return -1;
    }
    return 0;
}

// Given the two points for a new segment of a path (inf2 & inf3)
// as well as the distance between these points (dist), as well as
// possibly the previous point (inf1) [from inf1--inf2], return a
// cost associated with this route.
//
static double cost(
    ConnRef*     lineRef,
    const double dist,
    VertInf*     inf2,
    VertInf*     inf3,
    ANode*       inf1Node
)
{
    bool     isOrthogonal = (lineRef->routingType() == ConnType_Orthogonal);
    VertInf* inf1         = (inf1Node) ? inf1Node->inf : nullptr;
    double   result       = dist;
    Polygon  connRoute;

    Router* router = inf2->_router;
    if (inf1 != nullptr)
    {
        const double angle_penalty = router->routingParameter(anglePenalty);
        const double segmt_penalty = router->routingParameter(segmentPenalty);

        // This is not the first segment, so there is a bend
        // between it and the last one in the existing path.
        if ((angle_penalty > 0) || (segmt_penalty > 0))
        {
            Point p1 = inf1->point;
            Point p2 = inf2->point;
            Point p3 = inf3->point;

            double rad = M_PI - angleBetween(p1, p2, p3);

            if ((rad > 0) && !isOrthogonal)
            {
                // Make `xval' between 0--10 then take its log so small
                // angles are not penalised as much as large ones.
                //
                double xval  = rad * 10 / M_PI;
                double yval  = xval * log10(xval + 1) / 10.5;
                result      += (angle_penalty * yval);
                // db_printf("deg from straight: %g\tpenalty: %g\n",
                //         rad * 180 / M_PI, (angle_penalty * yval));
            }

            if (rad == M_PI)
            {
                // Needs to double back
                result += (2 * segmt_penalty);
            }
            else if (rad > 0)
            {
                // Only penalise as an extra segment if the two
                // segments are not collinear.
                result += segmt_penalty;
            }
        }
    }

    const double cluster_crossing_penalty
        = router->routingParameter(clusterCrossingPenalty);
    // XXX: Clustered routing doesn't yet work with orthogonal connectors.
    if (router->ClusteredRouting && !router->clusterRefs.empty()
        && (cluster_crossing_penalty > 0))
    {
        if (connRoute.empty())
        {
            constructPolygonPath(connRoute, inf2, inf3, inf1Node);
        }
        // There are clusters so do cluster routing.
        for (ClusterRefList::const_iterator cl = router->clusterRefs.begin();
             cl != router->clusterRefs.end();
             ++cl)
        {
            Polygon cBoundary = (isOrthogonal) ? (*cl)->rectangularPolygon()
                                               : (*cl)->polygon();
            if (cBoundary.size() <= 2)
            {
                continue;
            }
            COLA_ASSERT(cBoundary.ps[0] != cBoundary.ps[cBoundary.size() - 1]);
            for (size_t j = 0; j < cBoundary.size(); ++j)
            {
                // Non-orthogonal cluster boundary points should correspond to
                // shape vertices and hence already be in the list of vertices.
                COLA_ASSERT(
                    isOrthogonal
                    || router->vertices.getVertexByPos(cBoundary.at(j))
                );
            }

            bool               isConn = false;
            Polygon            dynamic_conn_route(connRoute);
            const bool         finalSegment = (inf3 == lineRef->dst());
            ConnectorCrossings cross(cBoundary, isConn, dynamic_conn_route);
            cross.checkForBranchingSegments = true;
            cross.countForSegment(connRoute.size() - 1, finalSegment);

            result += (cross.crossingCount * cluster_crossing_penalty);
        }
    }

    // This penalty penalises route segments that head in a direction opposite
    // of the direction(s) toward the target point.
    const double reversePenalty
        = router->routingParameter(reverseDirectionPenalty);
    if (reversePenalty)
    {
        // X and Y direction of destination from source point.
        const Point& srcPoint = lineRef->src()->point;
        const Point& dstPoint = lineRef->dst()->point;
        int          xDir     = dimDirection(dstPoint.x - srcPoint.x);
        int          yDir     = dimDirection(dstPoint.y - srcPoint.y);

        bool doesReverse = false;

        if ((xDir != 0)
            && (-xDir == dimDirection(inf3->point.x - inf2->point.x)))
        {
            // Connector has an X component and the segment heads in the
            // opposite direction.
            doesReverse |= true;
        }

        if ((yDir != 0)
            && (-yDir == dimDirection(inf3->point.y - inf2->point.y)))
        {
            // Connector has an Y component and the segment heads in the
            // opposite direction.
            doesReverse |= true;
        }

        if (doesReverse)
        {
            result += reversePenalty;
        }
    }

    if (!router->isInCrossingPenaltyReroutingStage())
    {
        // Return here if we are not in the post-processing stage
        return result;
    }

    const double crossing_penalty = router->routingParameter(crossingPenalty);
    const double shared_path_penalty
        = router->routingParameter(fixedSharedPathPenalty);
    if ((shared_path_penalty > 0) || (crossing_penalty > 0))
    {
        if (connRoute.empty())
        {
            constructPolygonPath(connRoute, inf2, inf3, inf1Node);
        }
        ConnRefList::const_iterator curr, finish = router->connRefs.end();
        for (curr = router->connRefs.begin(); curr != finish; ++curr)
        {
            ConnRef* connRef = *curr;

            if (connRef->id() == lineRef->id())
            {
                continue;
            }
            const Avoid::PolyLine& route2 = connRef->displayRoute();

            bool       isConn = true;
            Polygon    dynamic_route2(route2);
            Polygon    dynamic_conn_route(connRoute);
            const bool finalSegment = (inf3->point == lineRef->dst()->point);
            ConnectorCrossings cross(
                dynamic_route2,
                isConn,
                dynamic_conn_route,
                connRef,
                lineRef
            );
            cross.checkForBranchingSegments = true;
            cross.countForSegment(connRoute.size() - 1, finalSegment);

            if ((cross.crossingFlags & CROSSING_SHARES_PATH)
                && (cross.crossingFlags & CROSSING_SHARES_FIXED_SEGMENT)
                && (router->routingOption(
                        penaliseOrthogonalSharedPathsAtConnEnds
                    )
                    || !(cross.crossingFlags & CROSSING_SHARES_PATH_AT_END)))
            {
                // Penalise unnecessary shared paths in the middle of
                // connectors.
                result += shared_path_penalty;
            }
            result += (cross.crossingCount * crossing_penalty);
        }
    }

    return result;
}

// Directions for estimated orthgonal cost, as bitflags.
static const unsigned int CostDirectionN = 1;
static const unsigned int CostDirectionE = 2;
static const unsigned int CostDirectionS = 4;
static const unsigned int CostDirectionW = 8;

#ifdef ESTIMATED_COST_DEBUG

static void printDirections(FILE* fp, unsigned int directions)
{
    if (directions & CostDirectionN)
    {
        fprintf(fp, "N ");
    }
    if (directions & CostDirectionE)
    {
        fprintf(fp, "E ");
    }
    if (directions & CostDirectionS)
    {
        fprintf(fp, "S ");
    }
    if (directions & CostDirectionW)
    {
        fprintf(fp, "W ");
    }
}

#endif

// Returns the number of directions for the argument.
static unsigned int orthogonalDirectionsCount(const unsigned int directions)
{
    unsigned int count = 0;
    if (directions & CostDirectionN)
    {
        ++count;
    }
    if (directions & CostDirectionE)
    {
        ++count;
    }
    if (directions & CostDirectionS)
    {
        ++count;
    }
    if (directions & CostDirectionW)
    {
        ++count;
    }
    return count;
}

// Returns the directions of point b from point a.
static unsigned int orthogonalDirection(const Point& a, const Point& b)
{
    unsigned int result = 0;

    if (b.y > a.y)
    {
        result |= CostDirectionS;
    }
    else if (b.y < a.y)
    {
        result |= CostDirectionN;
    }

    if (b.x > a.x)
    {
        result |= CostDirectionE;
    }
    else if (b.x < a.x)
    {
        result |= CostDirectionW;
    }

    return result;
}

// Returns the direction to the right of the given direction.
static unsigned int dirRight(unsigned int direction)
{
    if (direction == CostDirectionN)
    {
        return CostDirectionE;
    }
    else if (direction == CostDirectionE)
    {
        return CostDirectionS;
    }
    else if (direction == CostDirectionS)
    {
        return CostDirectionW;
    }
    else if (direction == CostDirectionW)
    {
        return CostDirectionN;
    }

    // Should not be possible to reach here.
    COLA_ASSERT(false);
    return direction;
}

// Returns the direction to the left of the given direction.
static unsigned int dirLeft(unsigned int direction)
{
    if (direction == CostDirectionN)
    {
        return CostDirectionW;
    }
    else if (direction == CostDirectionE)
    {
        return CostDirectionN;
    }
    else if (direction == CostDirectionS)
    {
        return CostDirectionE;
    }
    else if (direction == CostDirectionW)
    {
        return CostDirectionS;
    }

    // Should not be possible to reach here.
    COLA_ASSERT(false);
    return direction;
}

// Returns the reverse direction to the given direction.
static unsigned int dirReverse(unsigned int direction)
{
    if (direction == CostDirectionN)
    {
        return CostDirectionS;
    }
    else if (direction == CostDirectionE)
    {
        return CostDirectionW;
    }
    else if (direction == CostDirectionS)
    {
        return CostDirectionN;
    }
    else if (direction == CostDirectionW)
    {
        return CostDirectionE;
    }

    // Should not be possible to reach here.
    COLA_ASSERT(false);
    return direction;
}

// Given Point curr with a direction of currDir, returns the nimimum number
// of bends to reach Point dest with the entry direction of destDir
//
// This is used for estimating the bend penalty cost to the target point
// from the current point of the search. The geometry was described in the
// "Orthogonal Connector Routing" paper, although the version described
// there is incorrect.
//
int bends(
    const Point& curr,
    unsigned int currDir,
    const Point& dest,
    unsigned int destDir
)
{
    // Bend counts from 'o' to 'D' should be:
    //
    //                1            1            3
    //                v            v            v
    //            2 > o < 2    2 > o < 2    4 > o < 2
    //                ^            ^            ^
    //                3            3            3
    //
    //   0 > o < 4                 D-->             4 > o < 4
    //       ^                                          ^
    //       1                                          3
    //
    COLA_ASSERT(currDir != 0);
    unsigned int currToDestDir  = orthogonalDirection(curr, dest);
    unsigned int reverseDestDir = dirReverse(destDir);
    bool         currDirPerpendicularToDestDir
        = (currDir == dirLeft(destDir)) || (currDir == dirRight(destDir));

    if ((currDir == destDir) && (currToDestDir == currDir))
    {
        //
        //   0 > o                     D-->
        //
        return 0;
    }
    else if (currDirPerpendicularToDestDir && (currToDestDir == (destDir | currDir)))
    {
        //
        //                1
        //                v
        //                o
        //
        //
        //                             D-->
        //
        return 1;
    }
    else if (currDirPerpendicularToDestDir && (currToDestDir == currDir))
    {
        //
        //                             1
        //                             v
        //                             o
        //
        //
        //                             D-->
        //
        return 1;
    }
    else if (currDirPerpendicularToDestDir && (currToDestDir == destDir))
    {
        //
        //       o                     D-->
        //       ^
        //       1
        //
        return 1;
    }
    else if ((currDir == destDir) && (currToDestDir != currDir) && !(currToDestDir & reverseDestDir))
    {
        //
        //            2 > o        2 > o
        //
        //
        //                             D-->
        //
        return 2;
    }
    else if (currDir == reverseDestDir && (currToDestDir != destDir) && (currToDestDir != currDir))
    {
        //
        //                o < 2        o < 2       o < 2
        //
        //
        //                             D-->
        //
        return 2;
    }
    else if (currDirPerpendicularToDestDir && (currToDestDir != (destDir | currDir)) && (currToDestDir != currDir))
    {
        //
        //                                          3
        //                                          v
        //                o            o            o
        //                ^            ^            ^
        //                3            3            3
        //
        //                             D-->                 o
        //                                                  ^
        //                                                  3
        //
        return 3;
    }
    else if ((currDir == reverseDestDir) && ((currToDestDir == destDir) || (currToDestDir == currDir)))
    {
        //
        //
        //
        //       o < 4                 D-->                 o < 4
        //
        return 4;
    }
    else if ((currDir == destDir) && (currToDestDir & reverseDestDir))
    {
        //
        //                                      4 > o
        //
        //
        //                             D-->             4 > o
        //
        return 4;
    }

    // Should not be possible to reach here.
    COLA_ASSERT(false);
    return 0;
}

static double estimatedCostSpecific(
    ConnRef*           lineRef,
    const Point*       last,
    const Point&       curr,
    const VertInf*     costTar,
    const unsigned int costTarDirs
)
{
    Point costTarPoint = costTar->point;

    if (lineRef->routingType() == ConnType_PolyLine)
    {
        return euclideanDist(curr, costTarPoint);
    }
    else  // Orthogonal
    {
        // Really doesn't make sense to route orthogonal paths without
        // a segment penalty.
        COLA_ASSERT(lineRef->router()->routingParameter(segmentPenalty) > 0);

        double dist = manhattanDist(curr, costTarPoint);

        int    bendCount = 0;
        double xmove     = costTarPoint.x - curr.x;
        double ymove     = costTarPoint.y - curr.y;
        if (last == nullptr)
        {
            // This is just the initial point.  Penalise it simply if it is
            // not inline with the target in either the x- or y-dimension.
            if ((xmove != 0) && (ymove != 0))
            {
                bendCount += 1;
            }
        }
        else if (dist > 0)
        {
            // We have two points and a non-zero distance, so we know
            // the segment direction.

            unsigned int currDir = orthogonalDirection(*last, curr);
            if ((currDir > 0) && (orthogonalDirectionsCount(currDir) == 1))
            {
                // Suitably high value, then we find the minimum.
                bendCount = 10;

                // Find the minimum bent penalty given all the possible
                // directions at the target point.
                if (costTarDirs & CostDirectionN)
                {
                    bendCount = std::min(
                        bendCount,
                        bends(curr, currDir, costTarPoint, CostDirectionN)
                    );
                }
                if (costTarDirs & CostDirectionE)
                {
                    bendCount = std::min(
                        bendCount,
                        bends(curr, currDir, costTarPoint, CostDirectionE)
                    );
                }
                if (costTarDirs & CostDirectionS)
                {
                    bendCount = std::min(
                        bendCount,
                        bends(curr, currDir, costTarPoint, CostDirectionS)
                    );
                }
                if (costTarDirs & CostDirectionW)
                {
                    bendCount = std::min(
                        bendCount,
                        bends(curr, currDir, costTarPoint, CostDirectionW)
                    );
                }
            }
        }
        double penalty
            = bendCount * lineRef->router()->routingParameter(segmentPenalty);

        return dist + penalty;
    }
}

static inline bool pointAlignedWithOneOf(
    const Point&              point,
    const std::vector<Point>& points,
    const size_t              dim
)
{
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (point[dim] == points[i][dim])
        {
            return true;
        }
    }
    return false;
}

AStarPath::AStarPath(void)
    : m_private(new AStarPathPrivate())
{
}

AStarPath::~AStarPath(void)
{
    delete m_private;
}

void AStarPath::search(
    ConnRef* lineRef,
    VertInf* src,
    VertInf* tar,
    VertInf* start
)
{
    m_private->search(lineRef, src, tar, start);
}

}  // namespace avoid
