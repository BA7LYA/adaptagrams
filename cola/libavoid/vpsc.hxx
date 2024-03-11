/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2005-2014  Monash University
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
 * Author(s):   Tim Dwyer
 *              Michael Wybrow
 *
 * --------------
 *
 * This file contains a slightly modified version of IncSolver() from libvpsc:
 * A solver for the problem of Variable Placement with Separation Constraints.
 * It has the following changes from the Adaptagrams VPSC version:
 *  -  The required VPSC code has been consolidated into a single file.
 *  -  Unnecessary code (like Solver) has been removed.
 *  -  The PairingHeap code has been replaced by a STL priority_queue.
 *
 * Modifications:  Michael Wybrow
 *
 */

#ifndef LIBAVOID_VPSC_H
#define LIBAVOID_VPSC_H

#ifdef USELIBVPSC

// By default, libavoid will use it's own version of VPSC defined in this file.
//
// Alternatively, you can directly use IncSolver from libvpsc.  This
// introduces a dependency on libvpsc but it can be preferable in cases
// where you are building all of Adaptagrams together and want to work
// with a set of CompoundConstraints or other classes built upon the
// base libvpsc Constraint classes.

// Include necessary headers from libvpsc.
#include "libvpsc/constraint.h"
#include "libvpsc/rectangle.h"
#include "libvpsc/solve_VPSC.h"
#include "libvpsc/variable.h"

// Use the libvpsc versions of things needed by libavoid.
using vpsc::Constraint;
using vpsc::Constraints;
using vpsc::delete_object;
using vpsc::IncSolver;
using vpsc::Variable;
using vpsc::Variables;

#else

#include <cfloat>
#include <iostream>
#include <list>
#include <queue>
#include <set>
#include <vector>

#include "libavoid/assertions.hxx"

namespace avoid {

class Variable;
using Variables = std::vector<Variable*>;

class Constraint;
using Constraints = std::vector<Constraint*>;

class CompareConstraints;

using Heap = std::
    priority_queue<Constraint*, std::vector<Constraint*>, CompareConstraints>;

struct delete_object
{
    template<typename T>
    void operator()(T* ptr)
    {
        delete ptr;
    }
};

extern Constraints constraintsRemovingRedundantEqualities(
    const Variables&   vars,
    const Constraints& constraints
);

}  // namespace avoid

#endif  // ! USELIBVPSC

#endif  // AVOID_VPSC_H
