/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2009-2013  Monash University
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

#ifndef AVOID_SCANLINE_H
#define AVOID_SCANLINE_H

#include <list>
#include <set>

namespace avoid {

class Router;
class ShiftSegmentList;

static const double CHANNEL_MAX = 1'0000'0000;

extern int  compare_events(const void* a, const void* b);
extern void buildConnectorRouteCheckpointCache(Router* router);
extern void clearConnectorRouteCheckpointCache(Router* router);
extern void buildOrthogonalChannelInfo(
    Router*           router,
    const size_t      dim,
    ShiftSegmentList& segmentList
);

}  // namespace avoid

#endif
