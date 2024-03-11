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

#include "libavoid/geomtypes.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdlib>

#include "libavoid/assertions.h"
#include "libavoid/router.h"
#include "libavoid/shape.h"

namespace avoid {

static Point unitNormalForEdge(const Point& pt1, const Point& pt2)
{
    if (pt2 == pt1)
    {
        return Point(0, 0);
    }
    double dx  = pt2.x - pt1.x;
    double dy  = pt2.y - pt1.y;
    double f   = 1.0 / std::sqrt((dx * dx) + (dy * dy));
    dx        *= f;
    dy        *= f;
    return Point(dy, -dx);
}

static const unsigned int SHORTEN_NONE  = 0;
static const unsigned int SHORTEN_START = 1;
static const unsigned int SHORTEN_END   = 2;
static const unsigned int SHORTEN_BOTH  = SHORTEN_START | SHORTEN_END;

// shorten_line():
//     Given the two endpoints of a line segment, this function adjusts the
//     endpoints of the line to shorten the line by shorten_length at either
//     or both ends.
//
static void shorten_line(
    double&            x1,
    double&            y1,
    double&            x2,
    double&            y2,
    const unsigned int mode,
    const double       shorten_length
)
{
    if (mode == SHORTEN_NONE)
    {
        return;
    }

    double rise  = y1 - y2;
    double run   = x1 - x2;
    double disty = fabs(rise);
    double distx = fabs(run);

    // Handle case where shorten length is greater than the length of the
    // line segment.
    if ((mode == SHORTEN_BOTH)
        && (((distx > disty) && ((shorten_length * 2) > distx))
            || ((disty >= distx) && ((shorten_length * 2) > disty))))
    {
        x1 = x2 = x1 - (run / 2);
        y1 = y2 = y1 - (rise / 2);
        return;
    }
    else if ((mode == SHORTEN_START) && (((distx > disty) && (shorten_length > distx)) || ((disty >= distx) && (shorten_length > disty))))
    {
        x1 = x2;
        y1 = y2;
        return;
    }
    else if ((mode == SHORTEN_END) && (((distx > disty) && (shorten_length > distx)) || ((disty >= distx) && (shorten_length > disty))))
    {
        x2 = x1;
        y2 = y1;
        return;
    }

    // Handle orthogonal line segments.
    if (x1 == x2)
    {
        // Vertical
        int sign = (y1 < y2) ? 1 : -1;

        if (mode & SHORTEN_START)
        {
            y1 += (sign * shorten_length);
        }
        if (mode & SHORTEN_END)
        {
            y2 -= (sign * shorten_length);
        }
        return;
    }
    else if (y1 == y2)
    {
        // Horizontal
        int sign = (x1 < x2) ? 1 : -1;

        if (mode & SHORTEN_START)
        {
            x1 += (sign * shorten_length);
        }
        if (mode & SHORTEN_END)
        {
            x2 -= (sign * shorten_length);
        }
        return;
    }

    int xpos = (x1 < x2) ? -1 : 1;
    int ypos = (y1 < y2) ? -1 : 1;

    double tangent = rise / run;

    if (mode & SHORTEN_END)
    {
        if (disty > distx)
        {
            y2 += shorten_length * ypos;
            x2 += shorten_length * ypos * (1 / tangent);
        }
        else if (disty < distx)
        {
            y2 += shorten_length * xpos * tangent;
            x2 += shorten_length * xpos;
        }
    }

    if (mode & SHORTEN_START)
    {
        if (disty > distx)
        {
            y1 -= shorten_length * ypos;
            x1 -= shorten_length * ypos * (1 / tangent);
        }
        else if (disty < distx)
        {
            y1 -= shorten_length * xpos * tangent;
            x1 -= shorten_length * xpos;
        }
    }
}

#define mid(a, b) ((a < b) ? a + ((b - a) / 2) : b + ((a - b) / 2))

}  // namespace avoid
