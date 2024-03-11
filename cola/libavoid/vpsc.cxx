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
 * Author(s):  Tim Dwyer
 *             Michael Wybrow
 *
 * --------------
 *
 * This file contains a slightly modified version of IncSolver() from libvpsc:
 * A solver for the problem of Variable Placement with Separation Constraints.
 * It has the following changes from the Adaptagrams VPSC version:
 *  -  The required VPSC code has been consolidated into a single file.
 *  -  Unnecessary code, like the Solver() class, has been removed.
 *  -  The PairingHeap code has been replaced by a STL priority_queue.
 *
 * Modifications:  Michael Wybrow
 *
 */

#include "libavoid/vpsc.h"

#ifndef USELIBVPSC

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <sstream>

#include "libavoid/assertions.hxx"
#include "libavoid/debug.h"

using namespace std;

namespace avoid {

static const double ZERO_UPPERBOUND      = -1e-10;
static const double LAGRANGIAN_TOLERANCE = -1e-4;

struct node
{
    set<node*> in;
    set<node*> out;
};

using std::copy;
using std::iterator;
using std::list;
using std::set;
using std::vector;

#define __NOTNAN(p) (p) == (p)

ostream& operator<<(ostream& os, const Block& b)
{
    os << "Block(posn=" << b.posn << "):";
    for (Block::Vit v = b.vars->begin(); v != b.vars->end(); ++v)
    {
        os << " " << **v;
    }
    if (b.deleted)
    {
        os << " Deleted!";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const Constraint& c)
{
    const char*        type = c.equality ? "=" : "<=";
    std::ostringstream lscale, rscale;
    if (c.left->scale != 1)
    {
        lscale << c.left->scale << "*";
    }
    if (c.right->scale != 1)
    {
        rscale << c.right->scale << "*";
    }
    os << lscale.str() << *c.left << "+" << c.gap << type << rscale.str()
       << *c.right;
    if (c.left->block && c.right->block)
    {
        os << "(" << c.slack() << ")" << (c.active ? "-active" : "")
           << "(lm=" << c.lm << ")";
    }
    else
    {
        os << "(vars have no position)";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const Variable& v)
{
    if (v.block)
    {
        os << "(" << v.id << "=" << v.position() << ")";
    }
    else
    {
        os << "(" << v.id << "=" << v.desiredPosition << ")";
    }
    return os;
}

typedef std::list<std::map<Variable*, double>> VarOffsetMapList;

Constraints constraintsRemovingRedundantEqualities(
    const Variables&   vars,
    const Constraints& constraints
)
{
    EqualityConstraintSet equalitySets(vars);
    Constraints           cs     = Constraints(constraints.size());
    int                   csSize = 0;

    for (unsigned i = 0; i < constraints.size(); ++i)
    {
        Constraint* c = constraints[i];
        if (c->equality)
        {
            if (!equalitySets.isRedundant(c->left, c->right, c->gap))
            {
                // Only add non-redundant equalities
                equalitySets.mergeSets(c->left, c->right, c->gap);
                cs[csSize++] = c;
            }
        }
        else
        {
            // Add all non-equalities
            cs[csSize++] = c;
        }
    }
    cs.resize(csSize);
    return cs;
}

}  // namespace avoid

#endif
