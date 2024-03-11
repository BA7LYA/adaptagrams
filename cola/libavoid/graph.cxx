/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2004-2011  Monash University
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

#include "libavoid/graph.h"

#include <cmath>

#include "libavoid/assertions.h"
#include "libavoid/connector.h"
#include "libavoid/debug.h"
#include "libavoid/geometry.h"
#include "libavoid/router.h"
#include "libavoid/timer.h"
#include "libavoid/vertices.h"

namespace avoid {

// Gives an order value between 0 and 3 for the point c, given the last
// segment was from a to b.  Returns the following value:
//    0 : Point c is directly backwards from point b.
//    1 : Point c is a left-hand 90 degree turn.
//    2 : Point c is a right-hand 90 degree turn.
//    3 : Point c is straight ahead (collinear).
//    4 : Point c is not orthogonally positioned.
//
static inline int orthogTurnOrder(
    const Point& a,
    const Point& b,
    const Point& c
)
{
    if (((c.x != b.x) && (c.y != b.y)) || ((a.x != b.x) && (a.y != b.y)))
    {
        // Not orthogonally positioned.
        return 4;
    }

    int direction = vecDir(a, b, c);

    if (direction > 0)
    {
        // Counterclockwise := left
        return 1;
    }
    else if (direction < 0)
    {
        // Clockwise := right
        return 2;
    }

    if (b.x == c.x)
    {
        if (((a.y < b.y) && (c.y < b.y)) || ((a.y > b.y) && (c.y > b.y)))
        {
            // Behind.
            return 0;
        }
    }
    else
    {
        if (((a.x < b.x) && (c.x < b.x)) || ((a.x > b.x) && (c.x > b.x)))
        {
            // Behind.
            return 0;
        }
    }

    // Ahead.
    return 3;
}

}  // namespace avoid
