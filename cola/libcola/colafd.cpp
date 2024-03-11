/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libcola - A library providing force-directed network layout using the
 *           stress-majorization method subject to separation constraints.
 *
 * Copyright (C) 2006-2015  Monash University
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * See the file LICENSE.LGPL distributed with the library.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author(s):  Tim Dwyer
 *             Michael Wybrow
 *
 */

// cmath needs ::strcpy_s under MinGW so include cstring.
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "libcola/cc_clustercontainmentconstraints.h"
#include "libcola/cc_nonoverlapconstraints.h"
#include "libcola/cola.h"
#include "libcola/commondefs.h"
#include "libcola/shortest_paths.h"
#include "libcola/straightener.h"
#include "libvpsc/constraint.h"
#include "libvpsc/exceptions.h"
#include "libvpsc/rectangle.h"
#include "libvpsc/solve_VPSC.h"
#include "libvpsc/variable.h"

#ifdef MAKEFEASIBLE_DEBUG
#include "libcola/output_svg.h"
#endif

// Needs to come last since it will include windows.h on WIN32 and
// may mess up C++ std library include on GCC 4.4
#include "libcola/cola_log.h"

using namespace std;

using vpsc::Constraint;
using vpsc::Constraints;
using vpsc::Dim;
using vpsc::IncSolver;
using vpsc::Rectangle;
using vpsc::Rectangles;
using vpsc::Variable;
using vpsc::Variables;
using vpsc::XDIM;
using vpsc::YDIM;

namespace cola {

template<class T>
void delete_vector(vector<T*>& v)
{
    for_each(v.begin(), v.end(), delete_object());
    v.clear();
}

Resizes PreIteration::__resizesNotUsed;
Locks   PreIteration::__locksNotUsed;

inline double dotProd(valarray<double> x, valarray<double> y)
{
    COLA_ASSERT(x.size() == y.size());
    double dp = 0;
    for (unsigned i = 0; i < x.size(); i++)
    {
        dp += x[i] * y[i];
    }
    return dp;
}

template<typename T>
void dumpSquareMatrix(unsigned n, T** L)
{
    printf("Matrix %dX%d\n{", n, n);
    for (unsigned i = 0; i < n; i++)
    {
        printf("{");
        for (unsigned j = 0; j < n; j++)
        {
            std::cout << L[i][j];
            char c = j == n - 1 ? '}' : ',';
            printf("%c", c);
        }
        char c = i == n - 1 ? '}' : ',';
        printf("%c\n", c);
    }
}

void dijkstra(
    const unsigned               s,
    const unsigned               n,
    double*                      d,
    const vector<Edge>&          es,
    const std::valarray<double>& eLengths
)
{
    shortest_paths::dijkstra(s, n, d, es, eLengths);
}

using Position = std::valarray<double>;

void getPosition(Position& X, Position& Y, Position& pos)
{
    unsigned n = X.size();
    COLA_ASSERT(Y.size() == n);
    COLA_ASSERT(pos.size() == 2 * n);
    for (unsigned i = 0; i < n; ++i)
    {
        pos[i]     = X[i];
        pos[i + n] = Y[i];
    }
}

// Used for sorting the CompoundConstraints from lowest priority to highest.
static bool cmpCompoundConstraintPriority(
    const cola::CompoundConstraint* lhs,
    const cola::CompoundConstraint* rhs
)
{
    return lhs->priority() < rhs->priority();
}

void setupVarsAndConstraints(
    unsigned                   n,
    const CompoundConstraints& ccs,
    const vpsc::Dim            dim,
    vpsc::Rectangles&          boundingBoxes,
    RootCluster*               clusterHierarchy,
    vpsc::Variables&           vs,
    vpsc::Constraints&         cs,
    valarray<double>&          coords
)
{
    vs.resize(n);
    for (unsigned i = 0; i < n; ++i)
    {
        vs[i] = new vpsc::Variable(i, coords[i]);
    }

    if (clusterHierarchy && !clusterHierarchy->flat())
    {
        // Create variables for clusters
        clusterHierarchy->computeBoundingRect(boundingBoxes);
        clusterHierarchy->createVars(dim, boundingBoxes, vs);
    }

    for (CompoundConstraints::const_iterator c = ccs.begin(); c != ccs.end();
         ++c)
    {
        (*c)->generateVariables(dim, vs);
    }
    for (CompoundConstraints::const_iterator c = ccs.begin(); c != ccs.end();
         ++c)
    {
        (*c)->generateSeparationConstraints(dim, vs, cs, boundingBoxes);
    }
}

static void setupExtraConstraints(
    const CompoundConstraints& ccs,
    const vpsc::Dim            dim,
    vpsc::Variables&           vs,
    vpsc::Constraints&         cs,
    vpsc::Rectangles&          boundingBoxes
)
{
    for (CompoundConstraints::const_iterator c = ccs.begin(); c != ccs.end();
         ++c)
    {
        (*c)->generateVariables(dim, vs);
    }
    for (CompoundConstraints::const_iterator c = ccs.begin(); c != ccs.end();
         ++c)
    {
        (*c)->generateSeparationConstraints(dim, vs, cs, boundingBoxes);
    }
}

void updateCompoundConstraints(
    const vpsc::Dim            dim,
    const CompoundConstraints& ccs
)
{
    for (CompoundConstraints::const_iterator c = ccs.begin(); c != ccs.end();
         ++c)
    {
        (*c)->updatePosition(dim);
    }
}

void project(
    vpsc::Variables&   vs,
    vpsc::Constraints& cs,
    valarray<double>&  coords
)
{
    unsigned        n = coords.size();
    vpsc::IncSolver s(vs, cs);
    s.solve();
    for (unsigned i = 0; i < n; ++i)
    {
        coords[i] = vs[i]->finalPosition;
    }
}

void setVariableDesiredPositions(
    vpsc::Variables&             vs,
    vpsc::Constraints&           cs,
    const DesiredPositionsInDim& des,
    valarray<double>&            coords
)
{
    COLA_UNUSED(cs);

    unsigned n = coords.size();
    COLA_ASSERT(vs.size() >= n);
    for (unsigned i = 0; i < n; ++i)
    {
        vpsc::Variable* v  = vs[i];
        v->desiredPosition = coords[i];
        v->weight          = 1;
    }
    for (DesiredPositionsInDim::const_iterator d = des.begin(); d != des.end();
         ++d)
    {
        COLA_ASSERT(d->first < vs.size());
        vpsc::Variable* v  = vs[d->first];
        v->desiredPosition = d->second;
        v->weight          = 1'0000;
    }
}

void checkUnsatisfiable(
    const vpsc::Constraints&      cs,
    UnsatisfiableConstraintInfos* unsatisfiable
)
{
    for (vpsc::Constraints::const_iterator c = cs.begin(); c != cs.end(); ++c)
    {
        if ((*c)->unsatisfiable)
        {
            UnsatisfiableConstraintInfo* i
                = new UnsatisfiableConstraintInfo(*c);
            unsatisfiable->push_back(i);
        }
    }
}

static const double LIMIT = 1'0000'0000;

static void reduceRange(double& val)
{
    val = std::min(val, LIMIT);
    val = std::max(val, -LIMIT);
}

ProjectionResult projectOntoCCs(
    Dim                 dim,
    Rectangles&         rs,
    CompoundConstraints ccs,
    bool                preventOverlaps,
    int                 accept,
    unsigned            debugLevel
)
{
    size_t                          n         = rs.size();
    // Set up nonoverlap constraints if desired.
    NonOverlapConstraintExemptions* nocexemps = nullptr;
    NonOverlapConstraints*          noc       = nullptr;
    if (preventOverlaps)
    {
        nocexemps = new NonOverlapConstraintExemptions();
        noc       = new NonOverlapConstraints(nocexemps);
        for (size_t i = 0; i < n; ++i)
        {
            noc->addShape(i, rs[i]->width() / 2.0, rs[i]->height() / 2.0);
        }
        ccs.push_back(noc);
    }
    // Set up vars and constraints.
    Variables   vs;
    Constraints cs;
    vs.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
        vs[i] = new Variable(i, rs[i]->getCentreD(dim));
    }
    for (CompoundConstraints::iterator it = ccs.begin(); it != ccs.end(); ++it)
    {
        CompoundConstraint* cc = *it;
        cc->generateVariables(dim, vs);
        cc->generateSeparationConstraints(dim, vs, cs, rs);
    }
    // Solve, if possible.
    ProjectionResult result = solve(vs, cs, rs, debugLevel);
    // If good enough, accept positions.
    if (result.errorLevel <= accept)
    {
        for (size_t i = 0; i < n; ++i)
        {
            rs[i]->moveCentreD(dim, vs[i]->finalPosition);
        }
    }
    // Clean up
    for (Variables::iterator it = vs.begin(); it != vs.end(); ++it)
    {
        delete *it;
    }
    for (Constraints::iterator it = cs.begin(); it != cs.end(); ++it)
    {
        delete *it;
    }
    delete noc;
    delete nocexemps;
    // Return
    return result;
}

ProjectionResult solve(
    Variables&   vs,
    Constraints& cs,
    Rectangles&  rs,
    unsigned     debugLevel
)
{
    int       result = 0;
    IncSolver solv(vs, cs);
    try
    {
        solv.solve();
    }
    catch (vpsc::UnsatisfiedConstraint uc)
    {
    }
    for (Constraints::iterator it = cs.begin(); it != cs.end(); it++)
    {
        Constraint* c = *it;
        if (c->unsatisfiable)
        {
            CompoundConstraint* cc = (CompoundConstraint*)(c->creator);
            if (cc->toString() == "NonOverlapConstraints()")
            {
                result = 1;
            }
            else
            {
                result = 2;
                break;
            }
        }
    }
    std::string unsatinfo;
    if (debugLevel > 0)
    {
        std::set<Variable*> varsInvolved;
        unsatinfo += "===================================================\n";
        unsatinfo += "UNSATISFIED CONSTRAINTS:\n";
        char buf[1000];
        for (Constraints::iterator it = cs.begin(); it != cs.end(); it++)
        {
            Constraint* c = *it;
            if (c->unsatisfiable)
            {
                varsInvolved.insert(c->left);
                varsInvolved.insert(c->right);
                sprintf(buf, "v_%d + %f", c->left->id, c->gap);
                unsatinfo += buf;
                unsatinfo += c->equality ? " == " : " <= ";
                sprintf(buf, "v_%d\n", c->right->id);
                unsatinfo += buf;
                if ((unsigned)c->left->id < rs.size())
                {
                    Rectangle* r = rs[c->left->id];
                    sprintf(
                        buf,
                        "    v_%d rect: [%f, %f] x [%f, %f]\n",
                        c->left->id,
                        r->getMinX(),
                        r->getMaxX(),
                        r->getMinY(),
                        r->getMaxY()
                    );
                    unsatinfo += buf;
                }
                if ((unsigned)c->right->id < rs.size())
                {
                    Rectangle* r = rs[c->right->id];
                    sprintf(
                        buf,
                        "    v_%d rect: [%f, %f] x [%f, %f]\n",
                        c->right->id,
                        r->getMinX(),
                        r->getMaxX(),
                        r->getMinY(),
                        r->getMaxY()
                    );
                    unsatinfo += buf;
                }
                CompoundConstraint* cc = (CompoundConstraint*)(c->creator);
                unsatinfo += "    Creator: " + cc->toString() + "\n";
            }
        }
        if (debugLevel > 1)
        {
            unsatinfo += "--------------------------------------------------\n";
            unsatinfo += "RELATED CONSTRAINTS:\n";
            std::set<Variable*>::iterator lit, rit, eit = varsInvolved.end();
            for (Constraints::iterator it = cs.begin(); it != cs.end(); it++)
            {
                Constraint* c = *it;
                lit           = varsInvolved.find(c->left);
                rit           = varsInvolved.find(c->right);
                if (lit != eit || rit != eit)
                {
                    sprintf(buf, "v_%d + %f", c->left->id, c->gap);
                    unsatinfo += buf;
                    unsatinfo += c->equality ? " == " : " <= ";
                    sprintf(buf, "v_%d\n", c->right->id);
                    unsatinfo += buf;
                    if ((unsigned)c->left->id < rs.size())
                    {
                        Rectangle* r = rs[c->left->id];
                        sprintf(
                            buf,
                            "    v_%d rect: [%f, %f] x [%f, %f]\n",
                            c->left->id,
                            r->getMinX(),
                            r->getMaxX(),
                            r->getMinY(),
                            r->getMaxY()
                        );
                        unsatinfo += buf;
                    }
                    if ((unsigned)c->right->id < rs.size())
                    {
                        Rectangle* r = rs[c->right->id];
                        sprintf(
                            buf,
                            "    v_%d rect: [%f, %f] x [%f, %f]\n",
                            c->right->id,
                            r->getMinX(),
                            r->getMaxX(),
                            r->getMinY(),
                            r->getMaxY()
                        );
                        unsatinfo += buf;
                    }
                    CompoundConstraint* cc = (CompoundConstraint*)(c->creator);
                    unsatinfo += "    Creator: " + cc->toString() + "\n";
                }
            }
        }
    }
    ProjectionResult pr;
    pr.errorLevel = result;
    pr.unsatinfo  = unsatinfo;
    return pr;
}

}  // namespace cola
