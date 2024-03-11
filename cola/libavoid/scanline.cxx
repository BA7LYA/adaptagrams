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

#include "libavoid/scanline.h"

#include <algorithm>
#include <cfloat>

#include "libavoid/connector.h"
#include "libavoid/junction.h"
#include "libavoid/obstacle.h"
#include "libavoid/router.h"
#include "libavoid/vertices.h"

namespace Avoid {

// Used for quicksort.  Must return <0, 0, or >0.
int compare_events(const void* a, const void* b)
{
    Event* ea = *(Event**)a;
    Event* eb = *(Event**)b;
    if (ea->pos != eb->pos)
    {
        return (ea->pos < eb->pos) ? -1 : 1;
    }
    if (ea->type != eb->type)
    {
        return ea->type - eb->type;
    }
    COLA_ASSERT(ea->v != eb->v);
    return (int)(ea->v - eb->v);
}

void buildConnectorRouteCheckpointCache(Router* router)
{
    for (ConnRefList::const_iterator curr = router->connRefs.begin();
         curr != router->connRefs.end();
         ++curr)
    {
        ConnRef* conn = *curr;
        if (conn->routingType() != ConnType_Orthogonal)
        {
            continue;
        }

        PolyLine&               displayRoute = conn->displayRoute();
        std::vector<Checkpoint> checkpoints  = conn->routingCheckpoints();

        // Initialise checkpoint vector and set to false.  There will be
        // one entry for each *segment* in the path, and the value indicates
        // whether the segment is affected by a checkpoint.
        displayRoute.checkpointsOnRoute
            = std::vector<std::pair<size_t, Point>>();

        for (size_t ind = 0; ind < displayRoute.size(); ++ind)
        {
            if (ind > 0)
            {
                for (size_t cpi = 0; cpi < checkpoints.size(); ++cpi)
                {
                    if (pointOnLine(
                            displayRoute.ps[ind - 1],
                            displayRoute.ps[ind],
                            checkpoints[cpi].point
                        ))
                    {
                        // The checkpoint is on a segment.
                        displayRoute.checkpointsOnRoute.push_back(
                            std::make_pair(
                                (ind * 2) - 1,
                                checkpoints[cpi].point
                            )
                        );
                    }
                }
            }

            for (size_t cpi = 0; cpi < checkpoints.size(); ++cpi)
            {
                if (displayRoute.ps[ind].equals(checkpoints[cpi].point))
                {
                    // The checkpoint is at a bendpoint.
                    displayRoute.checkpointsOnRoute.push_back(
                        std::make_pair(ind * 2, checkpoints[cpi].point)
                    );
                }
            }
        }
    }
}

void clearConnectorRouteCheckpointCache(Router* router)
{
    for (ConnRefList::const_iterator curr = router->connRefs.begin();
         curr != router->connRefs.end();
         ++curr)
    {
        ConnRef* conn = *curr;
        if (conn->routingType() != ConnType_Orthogonal)
        {
            continue;
        }

        // Clear the cache.
        PolyLine& displayRoute = conn->displayRoute();
        displayRoute.checkpointsOnRoute.clear();
    }
}

// Processes sweep events used to determine each horizontal and vertical
// line segment in a connector's channel of visibility.
// Four calls to this function are made at each position by the scanline:
//   1) Handle all Close event processing.
//   2) Remove Close event objects from the scanline.
//   3) Add Open event objects to the scanline.
//   4) Handle all Open event processing.
//
static void processShiftEvent(
    NodeSet&     scanline,
    Event*       e,
    size_t       dim,
    unsigned int pass
)
{
    Node* v = e->v;

    if (((pass == 3) && (e->type == Open))
        || ((pass == 3) && (e->type == SegOpen)))
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

    if (((pass == 4) && (e->type == Open))
        || ((pass == 4) && (e->type == SegOpen))
        || ((pass == 1) && (e->type == SegClose))
        || ((pass == 1) && (e->type == Close)))
    {
        if (v->ss)
        {
            // As far as we can see.
            double minLimit = v->firstObstacleAbove(dim);
            double maxLimit = v->firstObstacleBelow(dim);

            v->ss->minSpaceLimit = std::max(minLimit, v->ss->minSpaceLimit);
            v->ss->maxSpaceLimit = std::min(maxLimit, v->ss->maxSpaceLimit);
        }
        else
        {
            v->markShiftSegmentsAbove(dim);
            v->markShiftSegmentsBelow(dim);
        }
    }

    if (((pass == 2) && (e->type == SegClose))
        || ((pass == 2) && (e->type == Close)))
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

        size_t result;
        result = scanline.erase(v);
        COLA_ASSERT(result == 1);
        COLA_UNUSED(result);  // Avoid warning.
        delete v;
    }
}

void buildOrthogonalChannelInfo(
    Router*           router,
    const size_t      dim,
    ShiftSegmentList& segmentList
)
{
    if (segmentList.empty())
    {
        // There are no segments, so we can just return now.
        return;
    }

    // Do a sweep to determine space for shifting segments.
    size_t                 altDim      = (dim + 1) % 2;
    const size_t           n           = router->m_obstacles.size();
    const size_t           cpn         = segmentList.size();
    // Set up the events for the sweep.
    size_t                 totalEvents = 2 * (n + cpn);
    Event**                events      = new Event*[totalEvents];
    unsigned               ctr         = 0;
    ObstacleList::iterator obstacleIt  = router->m_obstacles.begin();
    for (unsigned i = 0; i < n; i++)
    {
        Obstacle*    obstacle = *obstacleIt;
        JunctionRef* junction = dynamic_cast<JunctionRef*>(obstacle);
        if (junction && !junction->positionFixed())
        {
            // Junctions that are free to move are not treated as obstacles.
            ++obstacleIt;
            totalEvents -= 2;
            continue;
        }
        Box    bBox   = obstacle->routingBox();
        Point  min    = bBox.min;
        Point  max    = bBox.max;
        double mid    = min[dim] + ((max[dim] - min[dim]) / 2);
        Node*  v      = new Node(obstacle, mid);
        events[ctr++] = new Event(Open, v, min[altDim]);
        events[ctr++] = new Event(Close, v, max[altDim]);

        ++obstacleIt;
    }
    for (ShiftSegmentList::iterator curr = segmentList.begin();
         curr != segmentList.end();
         ++curr)
    {
        const Point& lowPt  = (*curr)->lowPoint();
        const Point& highPt = (*curr)->highPoint();

        COLA_ASSERT(lowPt[dim] == highPt[dim]);
        COLA_ASSERT(lowPt[altDim] < highPt[altDim]);
        Node* v       = new Node(*curr, lowPt[dim]);
        events[ctr++] = new Event(SegOpen, v, lowPt[altDim]);
        events[ctr++] = new Event(SegClose, v, highPt[altDim]);
    }
    qsort((Event*)events, (size_t)totalEvents, sizeof(Event*), compare_events);

    // Process the sweep.
    // We do multiple passes over sections of the list so we can add relevant
    // entries to the scanline that might follow, before process them.
    NodeSet      scanline;
    double       thisPos        = (totalEvents > 0) ? events[0]->pos : 0;
    unsigned int posStartIndex  = 0;
    unsigned int posFinishIndex = 0;
    for (unsigned i = 0; i <= totalEvents; ++i)
    {
        // If we have finished the current scanline or all events, then we
        // process the events on the current scanline in a couple of passes.
        if ((i == totalEvents) || (events[i]->pos != thisPos))
        {
            posFinishIndex = i;
            for (int pass = 2; pass <= 4; ++pass)
            {
                for (unsigned j = posStartIndex; j < posFinishIndex; ++j)
                {
                    processShiftEvent(scanline, events[j], dim, pass);
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
        processShiftEvent(scanline, events[i], dim, pass);
    }
    COLA_ASSERT(scanline.size() == 0);
    for (unsigned i = 0; i < totalEvents; ++i)
    {
        delete events[i];
    }
    delete[] events;
}

}  // namespace Avoid
