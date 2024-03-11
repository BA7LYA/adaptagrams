/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2011-2015  Monash University
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

//! @file    hyperedge.h
//! @brief   Contains the interface for the HyperedgeRerouter class.

#ifndef AVOID_HYPEREDGE_H
#define AVOID_HYPEREDGE_H

#include <cstdio>
#include <list>
#include <set>
#include <vector>

#include "libavoid/dllexport.h"

namespace avoid {

class JunctionRef;
class Router;
class VertInf;

//! @brief   A list of JunctionRef objects.
using JunctionRefList       = std::list<JunctionRef*>;
using JunctionRefVector     = std::vector<JunctionRef*>;
using JunctionRefListVector = std::vector<JunctionRefList>;

using VertexList      = std::list<VertInf*>;
using VertexSet       = std::set<VertInf*>;
using VertexSetVector = std::vector<VertexSet>;

}  // namespace avoid

#endif
