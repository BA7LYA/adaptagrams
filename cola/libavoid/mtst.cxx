/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2011-2014  Monash University
 *
 * --------------------------------------------------------------------
 * Sequential Construction of the Minimum Terminal Spanning Tree is an
 * extended version of the method described in Section IV.B of:
 *     Long, J., Zhou, H., Memik, S.O. (2008). EBOARST: An efficient
 *     edge-based obstacle-avoiding rectilinear Steiner tree construction
 *     algorithm. IEEE Trans. on Computer-Aided Design of Integrated
 *     Circuits and Systems 27(12), pages 2169--2182.
 * --------------------------------------------------------------------
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

#include "libavoid/mtst.h"

#include <algorithm>
#include <cfloat>
#include <cstring>
#include <string>
#include <vector>

#include "libavoid/debughandler.h"
#include "libavoid/hyperedgetree.h"
#include "libavoid/junction.h"
#include "libavoid/router.h"
#include "libavoid/timer.h"
#include "libavoid/vertices.h"

namespace avoid {

struct delete_vertex
{
    void operator()(VertInf* ptr)
    {
        ptr->removeFromGraph(false);
        delete ptr;
    }
};

}  // namespace avoid
