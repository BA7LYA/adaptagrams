/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libtopology - Classes used in generating and managing topology constraints.
 *
 * Copyright (C) 2007-2008  Monash University
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * See the file LICENSE.LGPL distributed with the library.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author(s):  Tim Dwyer
 */

/*
 * Class definitions for graph elements used in determining topology
 * preserving constraints.
 */

#ifndef TOPOLOGY_GRAPH_H
#define TOPOLOGY_GRAPH_H

#include <functional>
#include <vector>

#include "libtopology/util.h"
#include "libvpsc/assertions.h"
#include "libvpsc/rectangle.h"

namespace vpsc {
class Variable;
}  // namespace vpsc

namespace topology {

class Segment;

/*
 * let n=ns.size(), where n<=vs.size(),
 * for i<n we set the variable for ns[i] to be vs[i].
 */
void setNodeVariables(Nodes& ns, std::vector<vpsc::Variable*>& vs);

// do nothing operator used in ForEach
template<typename T>
struct NoOp
{
    void operator()(T t)
    {
        COLA_UNUSED(t);
    }
};

/*
 * defines (hopefully just once) a loop over the bipartite linked-list
 * of Segment and EdgePoint in an Edge.
 * In the case of a cluster boundary, the edge will be a cycle, where
 * the last EdgePoint is also the first.  Thus, we process from
 * Edge::firstSegment to Edge::lastSegment.  We visit every EdgePoint
 * (i.e. nSegments+1), in the case of a cycle, the first/last
 * point will be visited (PointOp applied) twice unless noCycle is set
 * true.
 */
template<typename PEdge, typename PointOp, typename SegmentOp>
void ForEach(PEdge e, PointOp po, SegmentOp so, bool noCycle = false)
{
    Segment* s = e->firstSegment;
    if (!(e->cycle() && noCycle))
    {
        po(s->start);
    }
    bool last = false;
    do {
        EdgePoint* p = s->end;
        so(s);
        if (s == e->lastSegment)
        {
            last = true;
        }
        else
        {
            s = p->outSegment;
        }
        po(p);
    }
    while (!last);
}

double compute_stress(const Edges&);
void   printEdges(const Edges&);

/*
 * CrossProduct of three points: If the result is 0, the points are collinear;
 * if it is positive, the three points (in order) constitute a "left turn",
 * otherwise a "right turn".
 */
inline double crossProduct(
    double x0,
    double y0,
    double x1,
    double y1,
    double x2,
    double y2
)
{
    return (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
}

#ifndef NDEBUG
bool assertConvexBends(const Edges&);
/*
 * Asserts that there are no intersections between any of the segments
 * in edges and rectangles in nodes
 * @param nodes containing rectangles
 * @param edges containing segments
 * @return true if assertions succeed
 */
bool assertNoSegmentRectIntersection(const Nodes&, const Edges&);
bool assertNoZeroLengthEdgeSegments(const Edges& es);
#endif

}  // namespace topology

#endif  // TOPOLOGY_GRAPH_H
