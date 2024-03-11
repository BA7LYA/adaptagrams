/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2011-2013  Monash University
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

#ifndef AVOID_MTST_H
#define AVOID_MTST_H

#include <cstdio>
#include <list>
#include <set>
#include <utility>

#include "libavoid/hyperedgetree.h"
#include "libavoid/vertices.h"

namespace avoid {

class VertInf;
class Router;
class ConnRef;
class EdgeInf;

using VertexSetList = std::list<VertexSet>;

using LayeredOrthogonalEdge     = std::pair<EdgeInf*, VertInf*>;
using LayeredOrthogonalEdgeList = std::list<LayeredOrthogonalEdge>;

}  // namespace avoid

#endif
