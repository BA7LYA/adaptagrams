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

#include "libavoid/connector.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <queue>

#include "libavoid/assertions.h"
#include "libavoid/connend.h"
#include "libavoid/debug.h"
#include "libavoid/debughandler.h"
#include "libavoid/junction.h"
#include "libavoid/makepath.h"
#include "libavoid/router.h"
#include "libavoid/visibility.h"

namespace avoid {

Point midpoint(Point a, Point b)
{
    Point midpoint;

    midpoint.x = (a.x + b.x) / 2.0;
    midpoint.y = (a.y + b.y) / 2.0;

    return midpoint;
}

// Validates a bend point on a path to check it does not form a zigzag corner.
// a, b, c are consecutive points on the path.  d and e are b's neighbours,
// forming the shape corner d-b-e.
//
bool validateBendPoint(VertInf* aInf, VertInf* bInf, VertInf* cInf)
{
    if (bInf->id.isConnectionPin() || bInf->id.isConnCheckpoint())
    {
        // We shouldn't check connection pins or checkpoints.
        return true;
    }
    bool bendOkay = true;

    if ((aInf == nullptr) || (cInf == nullptr))
    {
        // Not a bendpoint, i.e., the end of the connector, so don't test.
        return bendOkay;
    }

    COLA_ASSERT(bInf != nullptr);
    VertInf* dInf = bInf->shPrev;
    VertInf* eInf = bInf->shNext;
    COLA_ASSERT(dInf != nullptr);
    COLA_ASSERT(eInf != nullptr);

    Point& a = aInf->point;
    Point& b = bInf->point;
    Point& c = cInf->point;
    Point& d = dInf->point;
    Point& e = eInf->point;

    if ((a == b) || (b == c))
    {
        return bendOkay;
    }

#ifdef PATHDEBUG
    db_printf("a=(%g, %g)\n", a.x, a.y);
    db_printf("b=(%g, %g)\n", b.x, b.y);
    db_printf("c=(%g, %g)\n", c.x, c.y);
    db_printf("d=(%g, %g)\n", d.x, d.y);
    db_printf("e=(%g, %g)\n", e.x, e.y);
#endif
    // Check angle:
    int abc = vecDir(a, b, c);
#ifdef PATHDEBUG
    db_printf("(abc == %d) ", abc);
#endif

    if (abc == 0)
    {
        // The three consecutive point on the path are in a line.
        // There should always be an equally short path that skips this
        // bend point, but this check is used during rubber-band routing
        // so we allow this case.
        bendOkay = true;
    }
    else  // (abc != 0)
    {
        COLA_ASSERT(vecDir(d, b, e) > 0);
        int abe = vecDir(a, b, e);
        int abd = vecDir(a, b, d);
        int bce = vecDir(b, c, e);
        int bcd = vecDir(b, c, d);
#ifdef PATHDEBUG
        db_printf(
            "&& (abe == %d) && (abd == %d) &&\n(bce == %d) && (bcd == %d)",
            abe,
            abd,
            bce,
            bcd
        );
#endif

        bendOkay = false;
        if (abe > 0)
        {
            if ((abc > 0) && (abd >= 0) && (bce >= 0))
            {
                bendOkay = true;
            }
        }
        else if (abd < 0)
        {
            if ((abc < 0) && (abe <= 0) && (bcd <= 0))
            {
                bendOkay = true;
            }
        }
    }
#ifdef PATHDEBUG
    db_printf("\n");
#endif
    return bendOkay;
}

// Returns a vertex number representing a point on the line between
// two shape corners, represented by p0 and p1.
//
static int midVertexNumber(const Point& p0, const Point& p1, const Point& c)
{
    if (c.vn != kUnassignedVertexNumber)
    {
        // The split point is a shape corner, so doesn't need its
        // vertex number adjusting.
        return c.vn;
    }
    if ((p0.vn >= 4) && (p0.vn < kUnassignedVertexNumber))
    {
        // The point next to this has the correct nudging direction,
        // so use that.
        return p0.vn;
    }
    if ((p1.vn >= 4) && (p1.vn < kUnassignedVertexNumber))
    {
        // The point next to this has the correct nudging direction,
        // so use that.
        return p1.vn;
    }
    if ((p0.vn < 4) && (p1.vn < 4))
    {
        if (p0.vn != p1.vn)
        {
            return p0.vn;
        }
        // Splitting between two ordinary shape corners.
        int vn_mid = std::min(p0.vn, p1.vn);
        if ((std::max(p0.vn, p1.vn) == 3) && (vn_mid == 0))
        {
            vn_mid = 3;  // Next vn is effectively 4.
        }
        return vn_mid + 4;
    }
    COLA_ASSERT((p0.x == p1.x) || (p0.y == p1.y));
    if (p0.vn != kUnassignedVertexNumber)
    {
        if (p0.x == p1.x)
        {
            if ((p0.vn == 2) || (p0.vn == 3))
            {
                return 6;
            }
            return 4;
        }
        else
        {
            if ((p0.vn == 0) || (p0.vn == 3))
            {
                return 7;
            }
            return 5;
        }
    }
    else if (p1.vn != kUnassignedVertexNumber)
    {
        if (p0.x == p1.x)
        {
            if ((p1.vn == 2) || (p1.vn == 3))
            {
                return 6;
            }
            return 4;
        }
        else
        {
            if ((p1.vn == 0) || (p1.vn == 3))
            {
                return 7;
            }
            return 5;
        }
    }

    // Shouldn't both be new (kUnassignedVertexNumber) points.
    db_printf(
        "midVertexNumber(): p0.vn and p1.vn both = "
        "kUnassignedVertexNumber\n"
    );
    db_printf("p0.vn %d p1.vn %d\n", p0.vn, p1.vn);
    return kUnassignedVertexNumber;
}

// Break up overlapping parallel segments that are not the same edge in
// the visibility graph, i.e., where one segment is a subsegment of another.
void splitBranchingSegments(
    Avoid::Polygon& poly,
    bool            polyIsConn,
    Avoid::Polygon& conn,
    const double    tolerance
)
{
    for (std::vector<Avoid::Point>::iterator i = conn.ps.begin();
         i != conn.ps.end();
         ++i)
    {
        if (i == conn.ps.begin())
        {
            // Skip the first point.
            // There are points-1 segments in a connector.
            continue;
        }

        for (std::vector<Avoid::Point>::iterator j = poly.ps.begin();
             j != poly.ps.end();)
        {
            if (polyIsConn && (j == poly.ps.begin()))
            {
                // Skip the first point.
                // There are points-1 segments in a connector.
                ++j;
                continue;
            }
            Point& c0 = *(i - 1);
            Point& c1 = *i;

            Point& p0 = (j == poly.ps.begin()) ? poly.ps.back() : *(j - 1);
            Point& p1 = *j;

            // Check the first point of the first segment.
            if (((i - 1) == conn.ps.begin())
                && pointOnLine(p0, p1, c0, tolerance))
            {
                // db_printf("add to poly %g %g\n", c0.x, c0.y);

                c0.vn = midVertexNumber(p0, p1, c0);
                j     = poly.ps.insert(j, c0);
                if (j != poly.ps.begin())
                {
                    --j;
                }
                continue;
            }
            // And the second point of every segment.
            if (pointOnLine(p0, p1, c1, tolerance))
            {
                // db_printf("add to poly %g %g\n", c1.x, c1.y);

                c1.vn = midVertexNumber(p0, p1, c1);
                j     = poly.ps.insert(j, c1);
                if (j != poly.ps.begin())
                {
                    --j;
                }
                continue;
            }

            // Check the first point of the first segment.
            if (polyIsConn && ((j - 1) == poly.ps.begin())
                && pointOnLine(c0, c1, p0, tolerance))
            {
                // db_printf("add to conn %g %g\n", p0.x, p0.y);

                p0.vn = midVertexNumber(c0, c1, p0);
                i     = conn.ps.insert(i, p0);
                continue;
            }
            // And the second point of every segment.
            if (pointOnLine(c0, c1, p1, tolerance))
            {
                // db_printf("add to conn %g %g\n", p1.x, p1.y);

                p1.vn = midVertexNumber(c0, c1, p1);
                i     = conn.ps.insert(i, p1);
            }
            ++j;
        }
    }
}

static int segDir(const Point& p1, const Point& p2)
{
    int result = 1;
    if (p1.x == p2.x)
    {
        if (p2.y > p1.y)
        {
            result = -1;
        }
    }
    else if (p1.y == p2.y)
    {
        if (p2.x < p1.x)
        {
            result = -1;
        }
    }
    return result;
}

static bool posInlineWithConnEndSegs(
    const double          pos,
    const size_t          dim,
    const Avoid::Polygon& poly,
    const Avoid::Polygon& conn
)
{
    size_t pLast = poly.size() - 1;
    size_t cLast = conn.size() - 1;
    if ((
            // Is inline with the beginning of the "poly" line
            ((pos == poly.ps[0][dim]) && (pos == poly.ps[1][dim])) ||
            // Is inline with the end of the "poly" line
            ((pos == poly.ps[pLast][dim]) && (pos == poly.ps[pLast - 1][dim]))
        )
        && (
            // Is inline with the beginning of the "conn" line
            ((pos == conn.ps[0][dim]) && (pos == conn.ps[1][dim])) ||
            // Is inline with the end of the "conn" line
            ((pos == conn.ps[cLast][dim]) && (pos == conn.ps[cLast - 1][dim]))
        ))
    {
        return true;
    }
    return false;
}

// Computes the *shared* length of these two shared paths.
//
static double pathLength(
    Avoid::Point** c_path,
    Avoid::Point** p_path,
    size_t         size
)
{
    double length = 0;

    for (unsigned int ind = 1; ind < size; ++ind)
    {
        if ((*(c_path[ind - 1]) == *(p_path[ind - 1]))
            && (*(c_path[ind]) == *(p_path[ind])))
        {
            // This segment is shared by both paths.
            //
            // This function will only be used for orthogonal paths, so we
            // can use Manhattan distance here since it will be faster to
            // compute.
            length += manhattanDist(*(c_path[ind - 1]), *(c_path[ind]));
        }
    }

    return length;
}

}  // namespace avoid
