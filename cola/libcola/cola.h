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
 */

#ifndef COLA_H
#define COLA_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <utility>
#include <valarray>
#include <vector>

#include "libcola/Cluster.hxx"
#include "libcola/ConstrainedMajorizationLayout.hxx"
#include "libcola/exceptions.h"
#include "libcola/gradient_projection.h"
#include "libcola/pseudorandom.h"
#include "libcola/straightener.h"

namespace vpsc {
class Rectangle;
}  // namespace vpsc

namespace topology {
class ColaTopologyAddon;
}  // namespace topology

namespace dialect {
class Graph;
}  // namespace dialect

/**
 * @namespace cola
 * @brief libcola: Force-directed network layout subject to
 *       separation constraints library.
 *
 * You should use COLA via an instance of the ConstrainedFDLayout class.
 */
namespace cola {

class NonOverlapConstraints;
class NonOverlapConstraintExemptions;
class Lock;
class Resize;
class DesiredPosition;
class PreIteration;
class TestConvergence;
class ConstrainedMajorizationLayout;
class TopologyAddonInterface;
class ConstrainedFDLayout;
class ProjectionResult;

//! @brief A vector of node Indexes.
using NodeIndexes = std::vector<unsigned>;

//! @brief A vector of NodeIndexes.
using ListOfNodeIndexes = std::vector<NodeIndexes>;

//! Edges are simply a pair of indices to entries in the Node vector
using Edge = std::pair<unsigned, unsigned>;

//! EdgeLengths is a vector of ideal lengths for edges corresponding to
//! edges in the edge list.
typedef std::vector<double> EdgeLengths;

#define StandardEdgeLengths EdgeLengths()

/*
 * desired positions which should override those computed by applying forces
 * are passed in for a set of nodes.  The first entry is the Node->id, the
 * second is the desired position.
 */
using DesiredPositionInDim  = std::pair<unsigned, double>;
using DesiredPositionsInDim = std::vector<DesiredPositionInDim>;

vpsc::Rectangle bounds(vpsc::Rectangles& rs);

/**
 * @brief Attempt to do a projection onto a vector of cola CompoundConstraints.
 * @param dim the dimension in which to perform the projection
 * @param rs the rectangles representing the nodes
 * @param ccs the constraints
 * @param preventOverlaps boolean saying whether you want overlap prevention
 *                        constraints to be automatically generated
 * @param accept  an integer indicating which types of infeasibilities you will
 * accept. The default value of 0 means you accept no infeasibility. For other
 * values, see the description of the "errorLevel" in the doctext for the solve
 * function below.
 * @param debugLevel see solve function below
 * @note          Rectangle positions are updated if and only if the error level
 * is less than or equal to the accept level.
 * @return a ProjectionResult indicating whether the projection was feasible or
 * not.
 * @sa solve
 */
ProjectionResult projectOntoCCs(
    vpsc::Dim                 dim,
    vpsc::Rectangles&         rs,
    cola::CompoundConstraints ccs,
    bool                      preventOverlaps,
    int                       accept     = 0,
    unsigned                  debugLevel = 0
);

/**
 * @brief Constructs a solver and attempts to solve the passed constraints on
 * the passed vars.
 * @param debugLevel: controls how much information comes back when the
 * projection fails. See below.
 * @return a ProjectionResult, containing:
 *  errorLevel:
 *   0: all constraints were satisfiable.
 *   1: some constraints were unsatisfiable, but they were all nonoverlap
 * constraints. 2: some constraints were unsatisfiable which were /not/
 * nonoverlap constraints. unsatinfo: The amount of information reported depends
 * on the debugLevel: 0: nothing reported (empty string) 1: description of the
 * unsatisfied constraints 2: the info from level 1, plus a description of all
 * "related" constraints (those sharing a variable). This is useful for
 * understanding the conflicts.
 */
ProjectionResult solve(
    vpsc::Variables&   vs,
    vpsc::Constraints& cs,
    vpsc::Rectangles&  rs,
    unsigned           debugLevel = 0
);

ConstrainedMajorizationLayout* simpleCMLFactory(
    vpsc::Rectangles&        rs,
    const std::vector<Edge>& es,
    RootCluster*             clusterHierarchy,
    const double             idealLength,
    bool                     useNeighbourStress = false
);

/*
 * find shortest path lengths from node s to all other nodes.
 * @param s starting node
 * @param n total number of nodes
 * @param d n vector of path lengths
 * @param es edge pairs
 * @param eLengths edge weights
 */
void dijkstra(
    const unsigned                 s,
    const unsigned                 n,
    double*                        d,
    const std::vector<cola::Edge>& es,
    const std::valarray<double>&   eLengths
);

#if 0
void removeClusterOverlapFast(RootCluster& clusterHierarchy, vpsc::Rectangles& rs, Locks& locks);
#endif

void setupVarsAndConstraints(
    unsigned                   n,
    const CompoundConstraints& ccs,
    const vpsc::Dim            dim,
    vpsc::Rectangles&          boundingBoxes,
    RootCluster*               clusterHierarchy,
    vpsc::Variables&           vs,
    vpsc::Constraints&         cs,
    std::valarray<double>&     coords
);
void setVariableDesiredPositions(
    vpsc::Variables&             vs,
    vpsc::Constraints&           cs,
    const DesiredPositionsInDim& des,
    std::valarray<double>&       coords
);

}  // namespace cola

#endif  // COLA_H
