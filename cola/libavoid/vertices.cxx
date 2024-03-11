/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2004-2009  Monash University
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

#include "libavoid/vertices.h"

#include <cstdlib>
#include <iostream>

#include "libavoid/assertions.h"
#include "libavoid/connend.h"
#include "libavoid/debug.h"
#include "libavoid/geometry.h"
#include "libavoid/graph.h"  // For alertConns
#include "libavoid/router.h"

using std::ostream;

namespace avoid {

ostream& operator<<(ostream& os, const VertID& vID)
{
    return os << '[' << vID.objID << ',' << vID.vn << ']';
}

bool directVis(VertInf* src, VertInf* dst)
{
    ShapeSet ss = ShapeSet();

    Point& p = src->point;
    Point& q = dst->point;

    VertID& pID = src->id;
    VertID& qID = dst->id;

    // We better be part of the same instance of libavoid.
    Router* router = src->_router;
    COLA_ASSERT(router == dst->_router);

    ContainsMap& contains = router->contains;
    if (pID.isConnPt())
    {
        ss.insert(contains[pID].begin(), contains[pID].end());
    }
    if (qID.isConnPt())
    {
        ss.insert(contains[qID].begin(), contains[qID].end());
    }

    // The "beginning" should be the first shape vertex, rather
    // than an endpoint, which are also stored in "vertices".
    VertInf* endVert = router->vertices.end();
    for (VertInf* k = router->vertices.shapesBegin(); k != endVert;
         k          = k->lstNext)
    {
        if ((ss.find(k->id.objID) == ss.end()))
        {
            if (segmentIntersect(p, q, k->point, k->shNext->point))
            {
                return false;
            }
        }
    }
    return true;
}

#define checkVertInfListConditions()                                           \
    do {                                                                       \
        COLA_ASSERT(                                                           \
            (!_firstConnVert && (_connVertices == 0))                          \
            || ((_firstConnVert->lstPrev == nullptr) && (_connVertices > 0))   \
        );                                                                     \
        COLA_ASSERT(                                                           \
            (!_firstShapeVert && (_shapeVertices == 0))                        \
            || ((_firstShapeVert->lstPrev == nullptr) && (_shapeVertices > 0)) \
        );                                                                     \
        COLA_ASSERT(!_lastShapeVert || (_lastShapeVert->lstNext == nullptr));  \
        COLA_ASSERT(                                                           \
            !_lastConnVert || (_lastConnVert->lstNext == _firstShapeVert)      \
        );                                                                     \
        COLA_ASSERT(                                                           \
            (!_firstConnVert && !_lastConnVert)                                \
            || (_firstConnVert && _lastConnVert)                               \
        );                                                                     \
        COLA_ASSERT(                                                           \
            (!_firstShapeVert && !_lastShapeVert)                              \
            || (_firstShapeVert && _lastShapeVert)                             \
        );                                                                     \
        COLA_ASSERT(!_firstShapeVert || !(_firstShapeVert->id.isConnPt()));    \
        COLA_ASSERT(!_lastShapeVert || !(_lastShapeVert->id.isConnPt()));      \
        COLA_ASSERT(!_firstConnVert || _firstConnVert->id.isConnPt());         \
        COLA_ASSERT(!_lastConnVert || _lastConnVert->id.isConnPt());           \
    }                                                                          \
    while (0)

}  // namespace avoid
