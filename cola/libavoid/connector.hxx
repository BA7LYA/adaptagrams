/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2004-2015  Monash University
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
 * Author(s):   Michael Wybrow
 */

//! @file    connector.h
//! @brief   Contains the interface for the ConnRef class.

#ifndef AVOID_CONNECTOR_H
#define AVOID_CONNECTOR_H

#include <list>
#include <utility>
#include <vector>

#include "libavoid/connend.h"
#include "libavoid/dllexport.h"
#include "libavoid/geometry.h"
#include "libavoid/vertices.h"

namespace avoid {

class Router;
class ConnRef;
class JunctionRef;
class ShapeRef;
using ConnRefList           = std::list<ConnRef*>;
using NodeIndexPairLinkList = std::list<std::pair<size_t, size_t>>;
using PointSet              = std::set<avoid::Point>;
using CrossingsInfoPair     = std::pair<int, unsigned int>;
using PointList             = std::vector<avoid::Point>;
using SharedPathList        = std::vector<PointList>;

const unsigned int CROSSING_NONE                 = 0;
const unsigned int CROSSING_TOUCHES              = 1;
const unsigned int CROSSING_SHARES_PATH          = 2;
const unsigned int CROSSING_SHARES_PATH_AT_END   = 4;
const unsigned int CROSSING_SHARES_FIXED_SEGMENT = 8;

extern void splitBranchingSegments(
    avoid::Polygon& poly,
    bool            polyIsConn,
    avoid::Polygon& conn,
    const double    tolerance = 0
);

extern bool validateBendPoint(VertInf* aInf, VertInf* bInf, VertInf* cInf);

}  // namespace avoid

#endif
