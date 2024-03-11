/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2009-2014  Monash University
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

#include "libavoid/orthogonal.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <list>
#include <set>

#include "libavoid/assertions.h"
#include "libavoid/connector.h"
#include "libavoid/connend.h"
#include "libavoid/debughandler.h"
#include "libavoid/geomtypes.h"
#include "libavoid/junction.h"
#include "libavoid/router.h"
#include "libavoid/scanline.h"
#include "libavoid/shape.h"
#include "libavoid/vpsc.h"

// For debugging:
// #define NUDGE_DEBUG
// #define DEBUG_JUST_UNIFY

namespace avoid {

// IDs:
static const int freeSegmentID  = 0;
static const int fixedSegmentID = 1;
static const int channelLeftID  = 2;
static const int channelRightID = 3;

// Weights:
static const double freeWeight     = 0.00001;
static const double strongWeight   = 0.001;
static const double strongerWeight = 1.0;
static const double fixedWeight    = 10'0000;

// Returns a bitfield of the directions of visibility in terms of the scanline
// in a particular dimension dimension.  It will return either ConnDirDown
// (meaning visibility to lower position values) or ConnDirUp (for visibility
// towards higher position values).
//
static ScanVisDirFlags getPosVertInfDirections(VertInf* v, size_t dim)
{
    if (dim == XDIM)  // X-dimension
    {
        unsigned int dirs = v->visDirections & (ConnDirLeft | ConnDirRight);
        if (dirs == (ConnDirLeft | ConnDirRight))
        {
            return (VisDirDown | VisDirUp);
        }
        else if (dirs == ConnDirLeft)
        {
            return VisDirDown;
        }
        else if (dirs == ConnDirRight)
        {
            return VisDirUp;
        }
    }
    else if (dim == YDIM)  // Y-dimension
    {
        unsigned int dirs = v->visDirections & (ConnDirDown | ConnDirUp);
        if (dirs == (ConnDirDown | ConnDirUp))
        {
            return (VisDirDown | VisDirUp);
        }
        else if (dirs == ConnDirDown)
        {
            // libavoid's Y-axis points downwards, so where the user has
            // specified visibility downwards, this results in visibility to
            // higher scanline positition values.
            return VisDirUp;
        }
        else if (dirs == ConnDirUp)
        {
            // libavoid's Y-axis points downwards, so where the user has
            // specified visibility upwards, this results in visibility to
            // lower scanline positition values.
            return VisDirDown;
        }
    }

    // Can occur for ConnDirNone visibility.
    return VisDirNone;
}

// A set of points to break the line segment,
// along with vertices for these points.
using BreakpointSet = std::set<PosVertInf>;

// Given a router instance and a set of possible horizontal segments, and a
// possible vertical visibility segment, compute and add edges to the
// orthogonal visibility graph for all the visibility edges.
static void intersectSegments(
    Router*      router,
    SegmentList& segments,
    LineSegment& vertLine
)
{
    // XXX: It seems that this case can sometimes occur... maybe when
    // there are many overlapping rectangles.
    // COLA_ASSERT(vertLine.beginVertInf() == nullptr);
    // COLA_ASSERT(vertLine.finishVertInf() == nullptr);

    COLA_ASSERT(!segments.empty());
    for (SegmentList::iterator it = segments.begin(); it != segments.end();)
    {
        LineSegment& horiLine = *it;

        bool inVertSegRegion
            = ((vertLine.begin <= horiLine.pos)
               && (vertLine.finish >= horiLine.pos));

        if (vertLine.pos < horiLine.begin)
        {
            // We've yet to reach this segment in the sweep, so ignore.
            ++it;
            continue;
        }
        else if (vertLine.pos == horiLine.begin)
        {
            if (inVertSegRegion)
            {
                horiLine.insertBreakpointsBegin(router, vertLine);
            }
        }
        else if (vertLine.pos == horiLine.finish)
        {
            if (inVertSegRegion)
            {
                // Add horizontal visibility segment.
                horiLine.addEdgeHorizontal(router);

                horiLine.insertBreakpointsFinish(router, vertLine);

                size_t dim = XDIM;  // x-dimension
                horiLine.generateVisibilityEdgesFromBreakpointSet(router, dim);

                // And we've now finished with the segment, so delete.
                it = segments.erase(it);
                continue;
            }
        }
        else if (vertLine.pos > horiLine.finish)
        {
            // Add horizontal visibility segment.
            horiLine.addEdgeHorizontal(router);

            size_t dim = XDIM;  // x-dimension
            horiLine.generateVisibilityEdgesFromBreakpointSet(router, dim);

            // We've now swept past this horizontal segment, so delete.
            it = segments.erase(it);
            continue;
        }
        else
        {
            COLA_ASSERT(vertLine.pos > horiLine.begin);
            COLA_ASSERT(vertLine.pos < horiLine.finish);

            if (inVertSegRegion)
            {
                // Add horizontal visibility segment.
                VertSet intersectionVerts
                    = horiLine.addEdgeHorizontalTillIntersection(
                        router,
                        vertLine
                    );

                for (VertSet::iterator v = intersectionVerts.begin();
                     v != intersectionVerts.end();
                     ++v)
                {
                    vertLine.breakPoints.insert(PosVertInf(
                        horiLine.pos,
                        *v,
                        getPosVertInfDirections(*v, YDIM)
                    ));
                }
            }
        }
        ++it;
    }

    // Split breakPoints set into visibility segments.
    size_t dimension = YDIM;  // y-dimension
    vertLine.generateVisibilityEdgesFromBreakpointSet(router, dimension);
}

// Processes an event for the vertical sweep used for computing the static
// orthogonal visibility graph.  This adds possible horizontal visibility
// segments to the segments list.
// The first pass is adding the event to the scanline, the second is for
// processing the event and the third for removing it from the scanline.
static void processEventVert(
    Router*             router,
    NodeSet&            scanline,
    SegmentListWrapper& segments,
    Event*              e,
    unsigned int        pass
)
{
    Node* v = e->v;

    if (((pass == 1) && (e->type == Open))
        || ((pass == 2) && (e->type == ConnPoint)))
    {
        std::pair<NodeSet::iterator, bool> result = scanline.insert(v);
        v->iter                                   = result.first;
        COLA_ASSERT(result.second);

        NodeSet::iterator it = v->iter;
        // Work out neighbours
        if (it != scanline.begin())
        {
            Node* u       = *(--it);
            v->firstAbove = u;
            u->firstBelow = v;
        }
        it = v->iter;
        if (++it != scanline.end())
        {
            Node* u       = *it;
            v->firstBelow = u;
            u->firstAbove = v;
        }
    }

    if (pass == 2)
    {
        if ((e->type == Open) || (e->type == Close))
        {
            // Only difference between Open and Close is whether the line
            // segments are at the top or bottom of the shape.  Decide here.
            double lineY = (e->type == Open) ? v->min[YDIM] : v->max[YDIM];

            // Shape edge positions.
            double minShape = v->min[XDIM];
            double maxShape = v->max[XDIM];
            // As far as we can see.
            double minLimit, maxLimit;
            double minLimitMax, maxLimitMin;
            v->findFirstPointAboveAndBelow(
                XDIM,
                lineY,
                minLimit,
                maxLimit,
                minLimitMax,
                maxLimitMin
            );

            // Insert possible visibility segments.
            if (minLimitMax >= maxLimitMin)
            {
                // These vertices represent the shape corners.
                VertInf* vI1 = new VertInf(
                    router,
                    dummyOrthogShapeID,
                    Point(minShape, lineY)
                );
                VertInf* vI2 = new VertInf(
                    router,
                    dummyOrthogShapeID,
                    Point(maxShape, lineY)
                );

                // There are no overlapping shapes, so give full visibility.
                if (minLimit < minShape)
                {
                    segments.insert(LineSegment(
                        minLimit,
                        minShape,
                        lineY,
                        true,
                        nullptr,
                        vI1
                    ));
                }
                segments.insert(
                    LineSegment(minShape, maxShape, lineY, true, vI1, vI2)
                );
                if (maxShape < maxLimit)
                {
                    segments.insert(LineSegment(
                        maxShape,
                        maxLimit,
                        lineY,
                        true,
                        vI2,
                        nullptr
                    ));
                }
            }
            else
            {
                // There are overlapping shapes along this shape edge.

                if ((minLimitMax > minLimit) && (minLimitMax >= minShape))
                {
                    LineSegment* line = segments.insert(
                        LineSegment(minLimit, minLimitMax, lineY, true)
                    );
                    // Shape corner:
                    VertInf* vI1 = new VertInf(
                        router,
                        dummyOrthogShapeID,
                        Point(minShape, lineY)
                    );
                    line->vertInfs.insert(vI1);
                }
                if ((maxLimitMin < maxLimit) && (maxLimitMin <= maxShape))
                {
                    LineSegment* line = segments.insert(
                        LineSegment(maxLimitMin, maxLimit, lineY, true)
                    );
                    // Shape corner:
                    VertInf* vI2 = new VertInf(
                        router,
                        dummyOrthogShapeID,
                        Point(maxShape, lineY)
                    );
                    line->vertInfs.insert(vI2);
                }
            }
        }
        else if (e->type == ConnPoint)
        {
            // Connection point.
            VertInf* centreVert = e->v->c;
            Point&   cp         = centreVert->point;

            // As far as we can see.
            double minLimit = v->firstPointAbove(XDIM);
            double maxLimit = v->firstPointBelow(XDIM);
            bool   inShape  = v->isInsideShape(XDIM);

            // Insert if we have visibility in that direction and the segment
            // length is greater than zero.
            LineSegment *line1 = nullptr, *line2 = nullptr;
            if ((centreVert->visDirections & ConnDirLeft) && (minLimit < cp.x))
            {
                line1 = segments.insert(LineSegment(
                    minLimit,
                    cp.x,
                    e->pos,
                    true,
                    nullptr,
                    centreVert
                ));
            }
            if ((centreVert->visDirections & ConnDirRight) && (cp.x < maxLimit))
            {
                line2 = segments.insert(LineSegment(
                    cp.x,
                    maxLimit,
                    e->pos,
                    true,
                    centreVert,
                    nullptr
                ));
                // If there was a line1, then we just merged with it, so
                // that pointer will be invalid (and now unnecessary).
                line1 = nullptr;
            }
            if (!line1 && !line2)
            {
                // Add a point segment for the centre point.
                segments.insert(LineSegment(cp.x, e->pos, centreVert));
            }

            if (!inShape)
            {
                // This is not contained within a shape so add a normal
                // visibility graph point here too (since paths won't route
                // *through* connector endpoint vertices).
                if (line1 || line2)
                {
                    VertInf* cent = new VertInf(router, dummyOrthogID, cp);
                    if (line1)
                    {
                        line1->vertInfs.insert(cent);
                    }
                    if (line2)
                    {
                        line2->vertInfs.insert(cent);
                    }
                }
            }
        }
    }

    if (((pass == 3) && (e->type == Close))
        || ((pass == 2) && (e->type == ConnPoint)))
    {
        // Clean up neighbour pointers.
        Node *l = v->firstAbove, *r = v->firstBelow;
        if (l != nullptr)
        {
            l->firstBelow = v->firstBelow;
        }
        if (r != nullptr)
        {
            r->firstAbove = v->firstAbove;
        }

        if (e->type == ConnPoint)
        {
            scanline.erase(v->iter);
            delete v;
        }
        else  // if (e->type == Close)
        {
            size_t result;
            result = scanline.erase(v);
            COLA_ASSERT(result == 1);
            COLA_UNUSED(result);  // Avoid warning.
            delete v;
        }
    }
}

// Processes an event for the vertical sweep used for computing the static
// orthogonal visibility graph.  This adds possible vertical visibility
// segments to the segments list.
// The first pass is adding the event to the scanline, the second is for
// processing the event and the third for removing it from the scanline.
static void processEventHori(
    Router*             router,
    NodeSet&            scanline,
    SegmentListWrapper& segments,
    Event*              e,
    unsigned int        pass
)
{
    Node* v = e->v;

    if (((pass == 1) && (e->type == Open))
        || ((pass == 2) && (e->type == ConnPoint)))
    {
        std::pair<NodeSet::iterator, bool> result = scanline.insert(v);
        v->iter                                   = result.first;
        COLA_ASSERT(result.second);

        NodeSet::iterator it = v->iter;
        // Work out neighbours
        if (it != scanline.begin())
        {
            Node* u       = *(--it);
            v->firstAbove = u;
            u->firstBelow = v;
        }
        it = v->iter;
        if (++it != scanline.end())
        {
            Node* u       = *it;
            v->firstBelow = u;
            u->firstAbove = v;
        }
    }

    if (pass == 2)
    {
        if ((e->type == Open) || (e->type == Close))
        {
            // Only difference between Open and Close is whether the line
            // segments are at the left or right of the shape.  Decide here.
            double lineX = (e->type == Open) ? v->min[XDIM] : v->max[XDIM];

            // Shape edge positions.
            double minShape = v->min[YDIM];
            double maxShape = v->max[YDIM];
            // As far as we can see.
            double minLimit, maxLimit;
            double minLimitMax, maxLimitMin;
            v->findFirstPointAboveAndBelow(
                YDIM,
                lineX,
                minLimit,
                maxLimit,
                minLimitMax,
                maxLimitMin
            );

            if (minLimitMax >= maxLimitMin)
            {
                LineSegment* line
                    = segments.insert(LineSegment(minLimit, maxLimit, lineX));

                // Shape corners:
                VertInf* vI1 = new VertInf(
                    router,
                    dummyOrthogShapeID,
                    Point(lineX, minShape)
                );
                VertInf* vI2 = new VertInf(
                    router,
                    dummyOrthogShapeID,
                    Point(lineX, maxShape)
                );
                line->vertInfs.insert(vI1);
                line->vertInfs.insert(vI2);
            }
            else
            {
                if ((minLimitMax > minLimit) && (minLimitMax >= minShape))
                {
                    LineSegment* line = segments.insert(
                        LineSegment(minLimit, minLimitMax, lineX)
                    );

                    // Shape corner:
                    VertInf* vI1 = new VertInf(
                        router,
                        dummyOrthogShapeID,
                        Point(lineX, minShape)
                    );
                    line->vertInfs.insert(vI1);
                }
                if ((maxLimitMin < maxLimit) && (maxLimitMin <= maxShape))
                {
                    LineSegment* line = segments.insert(
                        LineSegment(maxLimitMin, maxLimit, lineX)
                    );

                    // Shape corner:
                    VertInf* vI2 = new VertInf(
                        router,
                        dummyOrthogShapeID,
                        Point(lineX, maxShape)
                    );
                    line->vertInfs.insert(vI2);
                }
            }
        }
        else if (e->type == ConnPoint)
        {
            // Connection point.
            VertInf* centreVert = e->v->c;
            Point&   cp         = centreVert->point;

            // As far as we can see.
            double minLimit = v->firstPointAbove(YDIM);
            double maxLimit = v->firstPointBelow(YDIM);

            // Insert if we have visibility in that direction and the segment
            // length is greater than zero.
            if ((centreVert->visDirections & ConnDirUp) && (minLimit < cp.y))
            {
                segments.insert(LineSegment(minLimit, cp.y, e->pos));
            }

            if ((centreVert->visDirections & ConnDirDown) && (cp.y < maxLimit))
            {
                segments.insert(LineSegment(cp.y, maxLimit, e->pos));
            }
        }
    }

    if (((pass == 3) && (e->type == Close))
        || ((pass == 2) && (e->type == ConnPoint)))
    {
        // Clean up neighbour pointers.
        Node *l = v->firstAbove, *r = v->firstBelow;
        if (l != nullptr)
        {
            l->firstBelow = v->firstBelow;
        }
        if (r != nullptr)
        {
            r->firstAbove = v->firstAbove;
        }

        if (e->type == ConnPoint)
        {
            scanline.erase(v->iter);
            delete v;
        }
        else  // if (e->type == Close)
        {
            size_t result;
            result = scanline.erase(v);
            COLA_ASSERT(result == 1);
            COLA_UNUSED(result);  // Avoid warning.
            delete v;
        }
    }
}

// Correct visibility for pins or connector endpoints on the leading or
// trailing edge of the visibility graph which may only have visibility in
// the outward direction where there will not be a possible path.
void fixConnectionPointVisibilityOnOutsideOfVisibilityGraph(
    Event**      events,
    size_t       totalEvents,
    ConnDirFlags addedVisibility
)
{
    if (totalEvents > 0)
    {
        double firstPos = events[0]->pos;
        size_t index    = 0;
        while (index < totalEvents)
        {
            if (events[index]->pos > firstPos)
            {
                break;
            }

            if (events[index]->v->c)
            {
                events[index]->v->c->visDirections |= addedVisibility;
            }
            ++index;
        }
        index          = 0;
        double lastPos = events[totalEvents - 1]->pos;
        while (index < totalEvents)
        {
            size_t revIndex = totalEvents - 1 - index;
            if (events[revIndex]->pos < lastPos)
            {
                break;
            }

            if (events[revIndex]->v->c)
            {
                events[revIndex]->v->c->visDirections |= addedVisibility;
            }
            ++index;
        }
    }
}

extern void generateStaticOrthogonalVisGraph(Router* router)
{
    const size_t           n           = router->m_obstacles.size();
    const unsigned         cpn         = router->vertices.connsSize();
    // Set up the events for the vertical sweep.
    size_t                 totalEvents = (2 * n) + cpn;
    Event**                events      = new Event*[totalEvents];
    unsigned               ctr         = 0;
    ObstacleList::iterator obstacleIt  = router->m_obstacles.begin();
    for (unsigned i = 0; i < n; i++)
    {
        Obstacle* obstacle = *obstacleIt;
#ifndef PAPER
        JunctionRef* junction = dynamic_cast<JunctionRef*>(obstacle);
        if (junction && !junction->positionFixed())
        {
            // Junctions that are free to move are not treated as obstacles.
            ++obstacleIt;
            totalEvents -= 2;
            continue;
        }
#endif

        Box    bbox   = obstacle->routingBox();
        double midX   = bbox.min.x + ((bbox.max.x - bbox.min.x) / 2);
        Node*  v      = new Node(obstacle, midX);
        events[ctr++] = new Event(Open, v, bbox.min.y);
        events[ctr++] = new Event(Close, v, bbox.max.y);

        ++obstacleIt;
    }

#ifdef DEBUGHANDLER
    if (router->debugHandler())
    {
        std::vector<Box>       obstacleBoxes;
        ObstacleList::iterator obstacleIt = router->m_obstacles.begin();
        for (unsigned i = 0; i < n; i++)
        {
            Obstacle*    obstacle = *obstacleIt;
            JunctionRef* junction = dynamic_cast<JunctionRef*>(obstacle);
            if (junction && !junction->positionFixed())
            {
                // Junctions that are free to move are not treated as obstacles.
                ++obstacleIt;
                continue;
            }
            Box bbox = obstacle->routingBox();
            obstacleBoxes.push_back(bbox);
            ++obstacleIt;
        }
        router->debugHandler()->updateObstacleBoxes(obstacleBoxes);
    }
#endif

    for (VertInf* curr = router->vertices.connsBegin();
         curr && (curr != router->vertices.shapesBegin());
         curr = curr->lstNext)
    {
        if (curr->visDirections == ConnDirNone)
        {
            // This is a connector endpoint that is attached to a connection
            // pin on a shape, so it doesn't need to be given visibility.
            // Thus, skip it and record that there is one less total event.
            --totalEvents;
            continue;
        }
        Point& point = curr->point;

        Node* v       = new Node(curr, point.x);
        events[ctr++] = new Event(ConnPoint, v, point.y);
    }
    qsort((Event*)events, (size_t)totalEvents, sizeof(Event*), compare_events);

    // Correct visibility for pins or connector endpoints on the leading or
    // trailing edge of the visibility graph which may only have visibility in
    // the outward direction where there will not be a possible path.  We
    // fix this by giving them visibility left and right.
    fixConnectionPointVisibilityOnOutsideOfVisibilityGraph(
        events,
        totalEvents,
        (ConnDirLeft | ConnDirRight)
    );

    // Process the vertical sweep -- creating cadidate horizontal edges.
    // We do multiple passes over sections of the list so we can add relevant
    // entries to the scanline that might follow, before processing them.
    SegmentListWrapper segments;
    NodeSet            scanline;
    double             thisPos        = (totalEvents > 0) ? events[0]->pos : 0;
    unsigned int       posStartIndex  = 0;
    unsigned int       posFinishIndex = 0;
    for (unsigned i = 0; i <= totalEvents; ++i)
    {
        // Progress reporting and continuation check.
        router->performContinuationCheck(
            TransactionPhaseOrthogonalVisibilityGraphScanX,
            i,
            totalEvents
        );

        // If we have finished the current scanline or all events, then we
        // process the events on the current scanline in a couple of passes.
        if ((i == totalEvents) || (events[i]->pos != thisPos))
        {
            posFinishIndex = i;
            for (int pass = 2; pass <= 3; ++pass)
            {
                for (unsigned j = posStartIndex; j < posFinishIndex; ++j)
                {
                    processEventVert(
                        router,
                        scanline,
                        segments,
                        events[j],
                        pass
                    );
                }
            }

            if (i == totalEvents)
            {
                // We have cleaned up, so we can now break out of loop.
                break;
            }

            thisPos       = events[i]->pos;
            posStartIndex = i;
        }

        // Do the first sweep event handling -- building the correct
        // structure of the scanline.
        const int pass = 1;
        processEventVert(router, scanline, segments, events[i], pass);
    }
    COLA_ASSERT(scanline.size() == 0);
    for (unsigned i = 0; i < totalEvents; ++i)
    {
        delete events[i];
    }

    segments.list().sort();

    // Set up the events for the horizontal sweep.
    SegmentListWrapper vertSegments;
    ctr        = 0;
    obstacleIt = router->m_obstacles.begin();
    for (unsigned i = 0; i < n; i++)
    {
        Obstacle* obstacle = *obstacleIt;
#ifndef PAPER
        JunctionRef* junction = dynamic_cast<JunctionRef*>(obstacle);
        if (junction && !junction->positionFixed())
        {
            // Junctions that are free to move are not treated as obstacles.
            ++obstacleIt;
            continue;
        }
#endif
        Box    bbox   = obstacle->routingBox();
        double midY   = bbox.min.y + ((bbox.max.y - bbox.min.y) / 2);
        Node*  v      = new Node(obstacle, midY);
        events[ctr++] = new Event(Open, v, bbox.min.x);
        events[ctr++] = new Event(Close, v, bbox.max.x);

        ++obstacleIt;
    }
    for (VertInf* curr = router->vertices.connsBegin();
         curr && (curr != router->vertices.shapesBegin());
         curr = curr->lstNext)
    {
        if (curr->visDirections == ConnDirNone)
        {
            // This is a connector endpoint that is attached to a connection
            // pin on a shape, so it doesn't need to be given visibility.
            // Thus, skip it.
            continue;
        }
        Point& point = curr->point;

        Node* v       = new Node(curr, point.y);
        events[ctr++] = new Event(ConnPoint, v, point.x);
    }
    qsort((Event*)events, (size_t)totalEvents, sizeof(Event*), compare_events);

    // Correct visibility for pins or connector endpoints on the leading or
    // trailing edge of the visibility graph which may only have visibility in
    // the outward direction where there will not be a possible path.  We
    // fix this by giving them visibility up and down.
    fixConnectionPointVisibilityOnOutsideOfVisibilityGraph(
        events,
        totalEvents,
        (ConnDirUp | ConnDirDown)
    );

    // Process the horizontal sweep -- creating vertical visibility edges.
    thisPos       = (totalEvents > 0) ? events[0]->pos : 0;
    posStartIndex = 0;
    for (unsigned i = 0; i <= totalEvents; ++i)
    {
        // Progress reporting and continuation check.
        router->performContinuationCheck(
            TransactionPhaseOrthogonalVisibilityGraphScanY,
            i,
            totalEvents
        );

        // If we have finished the current scanline or all events, then we
        // process the events on the current scanline in a couple of passes.
        if ((i == totalEvents) || (events[i]->pos != thisPos))
        {
            posFinishIndex = i;
            for (int pass = 2; pass <= 3; ++pass)
            {
                for (unsigned j = posStartIndex; j < posFinishIndex; ++j)
                {
                    processEventHori(
                        router,
                        scanline,
                        vertSegments,
                        events[j],
                        pass
                    );
                }
            }

            // Process the merged line segments.
            vertSegments.list().sort();
            for (SegmentList::iterator curr = vertSegments.list().begin();
                 curr != vertSegments.list().end();
                 ++curr)
            {
                intersectSegments(router, segments.list(), *curr);
            }
            vertSegments.list().clear();

            if (i == totalEvents)
            {
                // We have cleaned up, so we can now break out of loop.
                break;
            }

            thisPos       = events[i]->pos;
            posStartIndex = i;
        }

        // Do the first sweep event handling -- building the correct
        // structure of the scanline.
        const int pass = 1;
        processEventHori(router, scanline, vertSegments, events[i], pass);
    }
    COLA_ASSERT(scanline.size() == 0);
    for (unsigned i = 0; i < totalEvents; ++i)
    {
        delete events[i];
    }
    delete[] events;

    // Add portions of horizontal lines that are after the final vertical
    // position we considered.
    for (SegmentList::iterator it = segments.list().begin();
         it != segments.list().end();)
    {
        LineSegment& horiLine = *it;

        horiLine.addEdgeHorizontal(router);

        size_t dim = XDIM;  // x-dimension
        horiLine.generateVisibilityEdgesFromBreakpointSet(router, dim);

        it = segments.list().erase(it);
    }
}

//============================================================================
//                           Path Adjustment code
//============================================================================

typedef std::pair<Point, Point> RectBounds;

static bool insideRectBounds(const Point& point, const RectBounds& rectBounds)
{
    Point zero(0, 0);
    if ((rectBounds.first == zero) && (rectBounds.second == zero))
    {
        // We can't be inside the invalid rectangle.
        return false;
    }

    for (size_t i = 0; i < 2; ++i)
    {
        if (point[i] < rectBounds.first[i])
        {
            return false;
        }
        if (point[i] > rectBounds.second[i])
        {
            return false;
        }
    }
    return true;
}

static void buildOrthogonalNudgingSegments(
    Router*           router,
    const size_t      dim,
    ShiftSegmentList& segmentList
)
{
    if (router->routingParameter(segmentPenalty) == 0)
    {
        // The nudging code assumes the routes are pretty optimal.  This will
        // only be true if a segment penalty is set, so just return if this
        // is not the case.
        return;
    }
    bool nudgeFinalSegments
        = router->routingOption(nudgeOrthogonalSegmentsConnectedToShapes);
    std::vector<RectBounds> shapeLimits;
    if (nudgeFinalSegments)
    {
        // If we're going to nudge final segments, then cache the shape
        // rectangles to save us rebuilding them multiple times.
        const size_t n = router->m_obstacles.size();
        shapeLimits    = std::vector<RectBounds>(n);

        double zeroBufferDist = 0.0;

        ObstacleList::iterator obstacleIt = router->m_obstacles.begin();
        for (unsigned i = 0; i < n; i++)
        {
            ShapeRef*    shape    = dynamic_cast<ShapeRef*>(*obstacleIt);
            JunctionRef* junction = dynamic_cast<JunctionRef*>(*obstacleIt);
            if (shape)
            {
                // Take the real bounds of the shape
                Box bBox = shape->polygon().offsetBoundingBox(zeroBufferDist);
                shapeLimits[i] = std::make_pair(bBox.min, bBox.max);
            }
            else if (junction)
            {
                // Don't nudge segments attached to junctions,
                // so just use the junction position here.
                Point pos      = junction->position();
                shapeLimits[i] = std::make_pair(pos, pos);
            }
            ++obstacleIt;
        }
    }

    size_t altDim = (dim + 1) % 2;
    // For each connector.
    for (ConnRefList::const_iterator curr = router->connRefs.begin();
         curr != router->connRefs.end();
         ++curr)
    {
        if ((*curr)->routingType() != ConnType_Orthogonal)
        {
            continue;
        }
        Polygon& displayRoute = (*curr)->displayRoute();
        // Determine all line segments that we are interested in shifting.
        // We don't consider the first or last segment of a path.
        for (size_t i = 1; i < displayRoute.size(); ++i)
        {
            if (displayRoute.ps[i - 1][dim] == displayRoute.ps[i][dim])
            {
                // It's a segment in the dimension we are processing,
                size_t indexLow  = i - 1;
                size_t indexHigh = i;
                if (displayRoute.ps[i - 1][altDim]
                    == displayRoute.ps[i][altDim])
                {
                    // This is a zero length segment, so ignore it.
                    continue;
                }
                else if (displayRoute.ps[i - 1][altDim] > displayRoute.ps[i][altDim])
                {
                    indexLow  = i;
                    indexHigh = i - 1;
                }

                // Find the checkpoints on the current segment and the
                // checkpoints on the adjoining segments that aren't on
                // the corner (hence the +1 and -1 modifiers).
                std::vector<Point> checkpoints
                    = displayRoute.checkpointsOnSegment(i - 1);
                std::vector<Point> prevCheckpoints
                    = displayRoute.checkpointsOnSegment(i - 2, -1);
                std::vector<Point> nextCheckpoints
                    = displayRoute.checkpointsOnSegment(i, +1);
                bool hasCheckpoints = (checkpoints.size() > 0);
                if (hasCheckpoints && !nudgeFinalSegments)
                {
                    // This segment includes one of the routing
                    // checkpoints so we shouldn't shift it.
                    segmentList.push_back(
                        new NudgingShiftSegment(*curr, indexLow, indexHigh, dim)
                    );
                    continue;
                }

                double thisPos = displayRoute.ps[i][dim];

                if ((i == 1) || ((i + 1) == displayRoute.size()))
                {
                    // Is first or last segment of route.

                    if (nudgeFinalSegments)
                    {
                        // Determine available space for nudging these
                        // final segments.
                        double minLim = -CHANNEL_MAX;
                        double maxLim = CHANNEL_MAX;

                        // If the position of the opposite end of the
                        // attached segment is within the shape boundaries
                        // then we want to use this as an ideal position
                        // for the segment.

                        // Bitflags indicating whether this segment starts
                        // and/or ends in a shape.
                        unsigned int endsInShapes = 0;
                        // Also limit their movement to the edges of the
                        // shapes they begin or end within.
                        for (size_t k = 0; k < shapeLimits.size(); ++k)
                        {
                            double shapeMin = shapeLimits[k].first[dim];
                            double shapeMax = shapeLimits[k].second[dim];
                            if (insideRectBounds(
                                    displayRoute.ps[i - 1],
                                    shapeLimits[k]
                                ))
                            {
                                minLim        = std::max(minLim, shapeMin);
                                maxLim        = std::min(maxLim, shapeMax);
                                endsInShapes |= 0x01;
                            }
                            if (insideRectBounds(
                                    displayRoute.ps[i],
                                    shapeLimits[k]
                                ))
                            {
                                minLim        = std::max(minLim, shapeMin);
                                maxLim        = std::min(maxLim, shapeMax);
                                endsInShapes |= 0x10;
                            }
                        }

                        if (endsInShapes == 0)
                        {
                            // If the segment is not within a shape, then we
                            // should limit it's nudging buffer so we don't
                            // combine many unnecessary regions.
                            double pos            = displayRoute.ps[i - 1][dim];
                            double freeConnBuffer = 15;
                            minLim = std::max(minLim, pos - freeConnBuffer);
                            maxLim = std::min(maxLim, pos + freeConnBuffer);
                        }

                        if ((minLim == maxLim) || (*curr)->hasFixedRoute())
                        {
                            // Fixed.
                            segmentList.push_back(new NudgingShiftSegment(
                                *curr,
                                indexLow,
                                indexHigh,
                                dim
                            ));
                        }
                        else
                        {
                            // Shiftable.
                            NudgingShiftSegment* segment
                                = new NudgingShiftSegment(
                                    *curr,
                                    indexLow,
                                    indexHigh,
                                    false,
                                    false,
                                    dim,
                                    minLim,
                                    maxLim
                                );
                            segment->finalSegment = true;
                            segment->endsInShape  = (endsInShapes > 0);
                            if ((displayRoute.size() == 2)
                                && (endsInShapes == 0x11))
                            {
                                // This is a single segment connector bridging
                                // two shapes.  So, we want to try to keep the
                                // segment centred rather than shift it.
                                segment->singleConnectedSegment = true;
                            }
                            segmentList.push_back(segment);
                        }
                    }
                    else
                    {
                        // The first and last segment of a connector can't be
                        // shifted.  We call them fixed segments.
                        segmentList.push_back(new NudgingShiftSegment(
                            *curr,
                            indexLow,
                            indexHigh,
                            dim
                        ));
                    }
                    continue;
                }

                // The segment probably has space to be shifted.
                double minLim = -CHANNEL_MAX;
                double maxLim = CHANNEL_MAX;

                // Constrain these segments by checkpoints along the
                // adjoining segments.  Ignore checkpoints at ends of
                // those segments.  XXX Perhaps this should not
                // affect the ideal centre position in the channel.
                for (size_t cp = 0; cp < nextCheckpoints.size(); ++cp)
                {
                    if (nextCheckpoints[cp][dim] < thisPos)
                    {
                        // Not at thisPoint, so constrain.
                        minLim = std::max(minLim, nextCheckpoints[cp][dim]);
                    }
                    else if (nextCheckpoints[cp][dim] > thisPos)
                    {
                        // Not at thisPoint, so constrain.
                        maxLim = std::min(maxLim, nextCheckpoints[cp][dim]);
                    }
                }
                for (size_t cp = 0; cp < prevCheckpoints.size(); ++cp)
                {
                    if (prevCheckpoints[cp][dim] < thisPos)
                    {
                        // Not at thisPoint, so constrain.
                        minLim = std::max(minLim, prevCheckpoints[cp][dim]);
                    }
                    else if (prevCheckpoints[cp][dim] > thisPos)
                    {
                        // Not at thisPoint, so constrain.
                        maxLim = std::min(maxLim, prevCheckpoints[cp][dim]);
                    }
                }

                bool isSBend = false;
                bool isZBend = false;

                if (checkpoints.empty())
                {
                    // Segments with checkpoints are held in place, but for
                    // other segments, we should limit their movement based
                    // on the limits of the segments at either end.

                    double prevPos = displayRoute.ps[i - 2][dim];
                    double nextPos = displayRoute.ps[i + 1][dim];
                    if (((prevPos < thisPos) && (nextPos > thisPos))
                        || ((prevPos > thisPos) && (nextPos < thisPos)))
                    {
                        // Determine limits if the s-bend is not due to an
                        // obstacle.  In this case we need to limit the channel
                        // to the span of the adjoining segments to this one.
                        if ((prevPos < thisPos) && (nextPos > thisPos))
                        {
                            minLim  = std::max(minLim, prevPos);
                            maxLim  = std::min(maxLim, nextPos);
                            isZBend = true;
                        }
                        else  // if ((prevPos > thisPos) && (nextPos < thisPos))
                        {
                            minLim  = std::max(minLim, nextPos);
                            maxLim  = std::min(maxLim, prevPos);
                            isSBend = true;
                        }
                    }
                }

                NudgingShiftSegment* nss = new NudgingShiftSegment(
                    *curr,
                    indexLow,
                    indexHigh,
                    isSBend,
                    isZBend,
                    dim,
                    minLim,
                    maxLim
                );
                nss->checkpoints = checkpoints;
                segmentList.push_back(nss);
            }
        }
    }
}

using ConnRefVector = std::vector<ConnRef*>;
using RouteVector   = std::vector<Polygon>;

// We can't use the normal sort algorithm for lists since it is not possible
// to compare all elements, but there will be an ordering defined between
// most of the elements.  Hence we order these, using insertion sort, and
// the case of them not being able to be compared is handled by not setting
// up any constraints between such segments when doing the nudging.
//
static ShiftSegmentList linesort(
    bool             nudgeFinalSegments,
    ShiftSegmentList origList,
    CmpLineOrder&    comparison
)
{
    // Cope with end segments that are getting moved and will line up with
    // other segments of the same connector.  We do this by merging them into
    // a single NudgingShiftSegment.
    if (nudgeFinalSegments)
    {
        for (ShiftSegmentList::iterator currSegIt = origList.begin();
             currSegIt != origList.end();
             ++currSegIt)
        {
            for (ShiftSegmentList::iterator otherSegIt = currSegIt;
                 otherSegIt != origList.end();)
            {
                NudgingShiftSegment* currSeg
                    = static_cast<NudgingShiftSegment*>(*currSegIt);
                NudgingShiftSegment* otherSeg
                    = static_cast<NudgingShiftSegment*>(*otherSegIt);
                if ((currSegIt != otherSegIt) && currSeg && otherSeg
                    && currSeg->shouldAlignWith(otherSeg, comparison.dimension))
                {
                    currSeg->mergeWith(otherSeg, comparison.dimension);
                    delete otherSeg;
                    otherSegIt = origList.erase(otherSegIt);
                }
                else
                {
                    ++otherSegIt;
                }
            }
        }
    }

    ShiftSegmentList resultList;

    size_t origListSize = origList.size();
    size_t deferredN    = 0;
    while (!origList.empty())
    {
        // Get and remove the first element from the origList.
        ShiftSegment* segment = origList.front();
        origList.pop_front();

        // Find the insertion point in the resultList.
        bool                       allComparable = true;
        ShiftSegmentList::iterator curr;
        for (curr = resultList.begin(); curr != resultList.end(); ++curr)
        {
            bool comparable  = false;
            bool lessThan    = comparison(segment, *curr, &comparable);
            allComparable   &= comparable;

            if (comparable && lessThan)
            {
                // If it is comparable and lessThan, then we have found the
                // insertion point.
                break;
            }
        }

        if (resultList.empty() || allComparable || (deferredN >= origListSize))
        {
            // Insert the element into the resultList at the required point.
            resultList.insert(curr, segment);
            // Reset the origListSize and deferred counter.
            deferredN    = 0;
            origListSize = origList.size();
        }
        else
        {
            // This wasn't comparable to anything in the sorted list,
            // so defer addition of the segment till later.
            origList.push_back(segment);
            deferredN++;
        }
    }

    return resultList;
}

using ShiftSegmentPtrList = std::list<ShiftSegment*>;

extern void improveOrthogonalRoutes(Router* router)
{
    ImproveOrthogonalRoutes improver(router);
    improver.execute();
}

}  // namespace avoid
