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
 * Author(s):   Michael Wybrow
 */

//! @file  geomtypes.h
//! @brief Contains the interface for various geometry types and classes.

#ifndef AVOID_GEOMTYPES_H
#define AVOID_GEOMTYPES_H

#include <cstdlib>
#include <utility>
#include <vector>

namespace avoid {

static const size_t XDIM = 0;
static const size_t YDIM = 1;

class Point;
class Polygon;
class ReferencingPolygon;
class Router;

//! @brief  A vector, represented by the Point class.
//!
using Vector = Point;

//! @brief  A multi-segment line, represented with the Polygon class.
//!
using PolyLine = Polygon;

//! Constant value representing an unassigned vertex number.
//!
static const unsigned short kUnassignedVertexNumber = 8;

//! Constant value representing a ShapeConnectionPin.
static const unsigned short kShapeConnectionPin = 9;

}  // namespace avoid

#endif
