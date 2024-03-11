/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2004-2013  Monash University
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

#ifndef AVOID_VERTICES_H
#define AVOID_VERTICES_H

#include <cstdio>
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <utility>

#include "libavoid/geomtypes.hxx"
#include "libavoid/VertID.hxx"

namespace avoid {

class VertInf;
class VertID;
using ShapeSet    = std::set<unsigned int>;
using ContainsMap = std::map<VertID, ShapeSet>;

// An ID given to all dummy vertices inserted to allow creation of the
// orthogonal visibility graph since the vertices in the orthogonal graph
// mostly do not correspond to shape corners or connector endpoints.
//
static const VertID dummyOrthogID(0, 0);
static const VertID dummyOrthogShapeID(0, 0, VertID::PROP_OrthShapeEdge);

// Orthogonal visibility property flags
static const unsigned int XL_EDGE = 1;
static const unsigned int XL_CONN = 2;
static const unsigned int XH_EDGE = 4;
static const unsigned int XH_CONN = 8;
static const unsigned int YL_EDGE = 16;
static const unsigned int YL_CONN = 32;
static const unsigned int YH_EDGE = 64;
static const unsigned int YH_CONN = 128;

bool directVis(VertInf* src, VertInf* dst);

}  // namespace avoid

#endif
