///
/// @file ImproveOrthogonalRoutes.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/ImproveOrthogonalRoutes.hxx"

namespace avoid {

ImproveOrthogonalRoutes::ImproveOrthogonalRoutes(Router* router)
    : m_router(router)
{
}

void ImproveOrthogonalRoutes::execute(void)
{
    TIMER_START(m_router, tmOrthogNudge);

    m_shared_path_connectors_with_common_endpoints.clear();

    // Simplify routes.
    simplifyOrthogonalRoutes();

    // Build a cache that denotes whether a certain segment of a connector
    // contains a checkpoint.  We can't just compare positions, since routes
    // can be moved away from their original positions during nudging.
    buildConnectorRouteCheckpointCache(m_router);

    // Do Unifying first, by itself.  This greedily tries to position free
    // segments in overlapping channels at the same position.  This way they
    // have correct nudging orders determined for them since they will form
    // shared paths, rather than segments just positioned as an results of
    // the routing process.  Of course, don't do this when rerouting with
    // a fixedSharedPathPenalty since these routes include extra segments
    // we want to keep apart which prevent some shared paths.
    if (m_router->routingOption(performUnifyingNudgingPreprocessingStep)
        && (m_router->routingParameter(fixedSharedPathPenalty) == 0))
    {
        for (size_t dimension = 0; dimension < 2; ++dimension)
        {
            // Just perform Unifying operation.
            bool justUnifying = true;
            m_segment_list.clear();
            buildOrthogonalNudgingSegments(m_router, dimension, m_segment_list);
            buildOrthogonalChannelInfo(m_router, dimension, m_segment_list);
            nudgeOrthogonalRoutes(dimension, justUnifying);
        }
    }

#ifndef DEBUG_JUST_UNIFY
    // Do the Nudging and centring.
    for (size_t dimension = 0; dimension < 2; ++dimension)
    {
        m_point_orders.clear();
        // Build nudging info.
        // XXX Needs to be rebuilt for each dimension, cause of shifting
        //     points.  Maybe we could modify the point orders.
        buildOrthogonalNudgingOrderInfo();

        // Do the centring and nudging.
        m_segment_list.clear();
        buildOrthogonalNudgingSegments(m_router, dimension, m_segment_list);
        buildOrthogonalChannelInfo(m_router, dimension, m_segment_list);
        nudgeOrthogonalRoutes(dimension);
    }
#endif  // DEBUG_JUST_UNIFY

    // Resimplify all the display routes that may have been split.
    simplifyOrthogonalRoutes();

    m_router->improveOrthogonalTopology();

    // Clear the segment-checkpoint cache for connectors.
    clearConnectorRouteCheckpointCache(m_router);

    TIMER_STOP(m_router);
}

void ImproveOrthogonalRoutes::nudgeOrthogonalRoutes(
    size_t dimension,
    bool   justUnifying
)
{
    bool nudgeFinalSegments
        = m_router->routingOption(nudgeOrthogonalSegmentsConnectedToShapes);
    bool nudgeSharedPathsWithCommonEnd
        = m_router->routingOption(nudgeSharedPathsWithCommonEndPoint);
    double baseSepDist = m_router->routingParameter(idealNudgingDistance);
    COLA_ASSERT(baseSepDist >= 0);
    // If we can fit things with the desired separation distance, then
    // we try 10 times, reducing each time by a 10th of the original amount.
    double reductionSteps = 10.0;

    size_t           totalSegmentsToShift = m_segment_list.size();
    size_t           numOfSegmentsShifted = 0;
    // Do the actual nudging.
    ShiftSegmentList currentRegion;
    while (!m_segment_list.empty())
    {
        // Progress reporting and continuation check.
        numOfSegmentsShifted = totalSegmentsToShift - m_segment_list.size();
        m_router->performContinuationCheck(
            (dimension == XDIM) ? TransactionPhaseOrthogonalNudgingX
                                : TransactionPhaseOrthogonalNudgingY,
            numOfSegmentsShifted,
            totalSegmentsToShift
        );

        // Take a reference segment
        ShiftSegment* currentSegment = m_segment_list.front();
        // Then, find the segments that overlap this one.
        currentRegion.clear();
        currentRegion.push_back(currentSegment);
        m_segment_list.erase(m_segment_list.begin());
        for (ShiftSegmentList::iterator curr = m_segment_list.begin();
             curr != m_segment_list.end();)
        {
            bool overlaps = false;
            for (ShiftSegmentList::iterator curr2 = currentRegion.begin();
                 curr2 != currentRegion.end();
                 ++curr2)
            {
                if ((*curr)->overlapsWith(*curr2, dimension))
                {
                    overlaps = true;
                    break;
                }
            }
            if (overlaps)
            {
                currentRegion.push_back(*curr);
                m_segment_list.erase(curr);
                // Consider segments from the beginning, since we may have
                // since passed segments that overlap with the new set.
                curr = m_segment_list.begin();
            }
            else
            {
                ++curr;
            }
        }

        if (!justUnifying)
        {
            CmpLineOrder lineSortComp(m_point_orders, dimension);
            currentRegion
                = linesort(nudgeFinalSegments, currentRegion, lineSortComp);
        }

        if (currentRegion.size() == 1)
        {
            // Save creating the solver instance if there is just one
            // immovable segment, or if we are in the unifying stage.
            if (currentRegion.front()->immovable() || justUnifying)
            {
                delete currentRegion.front();
                continue;
            }
        }

        // Process these segments.
        std::list<size_t>   freeIndexes;
        Variables           vs;
        Constraints         cs;
        Constraints         gapcs;
        ShiftSegmentPtrList prevVars;
        double              sepDist = baseSepDist;
#ifdef NUDGE_DEBUG
        fprintf(
            stderr,
            "-------------------------------------------------------\n"
        );
        fprintf(
            stderr,
            "%s -- size: %d\n",
            (justUnifying) ? "Unifying" : "Nudging",
            (int)currentRegion.size()
        );
#endif
#ifdef NUDGE_DEBUG_SVG
        printf("\n\n");
#endif
        for (ShiftSegmentList::iterator currSegmentIt = currentRegion.begin();
             currSegmentIt != currentRegion.end();
             ++currSegmentIt)
        {
            NudgingShiftSegment* currSegment
                = static_cast<NudgingShiftSegment*>(*currSegmentIt);

            // Create a solver variable for the position of this segment.
            currSegment->createSolverVariable(justUnifying);

            vs.push_back(currSegment->variable);
            size_t index = vs.size() - 1;
#ifdef NUDGE_DEBUG
            fprintf(
                stderr,
                "line(%d)  %.15f  dim: %d pos: %.16f\n"
                "min: %.16f  max: %.16f\n"
                "minEndPt: %.16f  maxEndPt: %.16f weight: %g cc: %d\n",
                currSegment->connRef->id(),
                currSegment->lowPoint()[dimension],
                (int)dimension,
                currSegment->variable->desiredPosition,
                currSegment->minSpaceLimit,
                currSegment->maxSpaceLimit,
                currSegment->lowPoint()[!dimension],
                currSegment->highPoint()[!dimension],
                currSegment->variable->weight,
                (int)currSegment->checkpoints.size()
            );
#endif
#ifdef NUDGE_DEBUG_SVG
            // Debugging info:
            double minP = std::max(currSegment->minSpaceLimit, -5000.0);
            double maxP = std::min(currSegment->maxSpaceLimit, 5000.0);
            fprintf(
                stdout,
                "<rect style=\"fill: #f00; opacity: 0.2;\" "
                "x=\"%g\" y=\"%g\" width=\"%g\" height=\"%g\" />\n",
                currSegment->lowPoint()[XDIM],
                minP,
                currSegment->highPoint()[XDIM] - currSegment->lowPoint()[XDIM],
                maxP - minP
            );
            fprintf(
                stdout,
                "<line style=\"stroke: #000;\" x1=\"%g\" "
                "y1=\"%g\" x2=\"%g\" y2=\"%g\" />\n",
                currSegment->lowPoint()[XDIM],
                currSegment->lowPoint()[YDIM],
                currSegment->highPoint()[XDIM],
                currSegment->highPoint()[YDIM]
            );
#endif

            if (justUnifying)
            {
                // Just doing centring, not nudging.
                // Record the index of the variable so we can use it as
                // a segment to potentially constrain to other segments.
                if (currSegment->variable->weight == freeWeight)
                {
                    freeIndexes.push_back(index);
                }
                // Thus, we don't need to constrain position against other
                // segments.
                prevVars.push_back(&(*currSegment));
                continue;
            }

            // The constraints generated here must be in order of
            // leftBoundary-segment ... segment-segment ...
            // segment-rightBoundary since this order is leveraged later for
            // rewriting the separations of unsatisfable channel groups.

            // Constrain to channel boundary.
            if (!currSegment->fixed)
            {
                // If this segment sees a channel boundary to its left,
                // then constrain its placement as such.
                if (currSegment->minSpaceLimit > -CHANNEL_MAX)
                {
                    vs.push_back(new Variable(
                        channelLeftID,
                        currSegment->minSpaceLimit,
                        fixedWeight
                    ));
                    cs.push_back(
                        new Constraint(vs[vs.size() - 1], vs[index], 0.0)
                    );
                }
            }

            // Constrain position in relation to previously seen segments,
            // if necessary (i.e. when they could overlap).
            for (ShiftSegmentPtrList::iterator prevVarIt = prevVars.begin();
                 prevVarIt != prevVars.end();
                 ++prevVarIt)
            {
                NudgingShiftSegment* prevSeg
                    = static_cast<NudgingShiftSegment*>(*prevVarIt);
                Variable* prevVar = prevSeg->variable;

                if (currSegment->overlapsWith(prevSeg, dimension)
                    && (!(currSegment->fixed) || !(prevSeg->fixed)))
                {
                    // If there is a previous segment to the left that
                    // could overlap this in the shift direction, then
                    // constrain the two segments to be separated.
                    // Though don't add the constraint if both the
                    // segments are fixed in place.
                    double thisSepDist = sepDist;
                    bool   equality    = false;
                    if (currSegment->shouldAlignWith(prevSeg, dimension))
                    {
                        // Handles the case where the two end segments can
                        // be brought together to make a single segment. This
                        // can help in situations where having the small kink
                        // can restrict other kinds of nudging.
                        thisSepDist = 0;
                        equality    = true;
                    }
                    else if (currSegment->canAlignWith(prevSeg, dimension))
                    {
                        // We need to address the problem of two neighbouring
                        // segments of the same connector being kept separated
                        // due only to a kink created in the other dimension.
                        // Here, we let such segments drift back together.
                        thisSepDist = 0;
                    }
                    else if (!nudgeSharedPathsWithCommonEnd &&
                            (m_shared_path_connectors_with_common_endpoints.count(
                                 UnsignedPair(currSegment->connRef->id(), prevSeg->connRef->id())) > 0))
                    {
                        // We don't want to nudge apart these two segments
                        // since they are from a shared path with a common
                        // endpoint.  There might be multiple chains of
                        // segments that don't all have the same endpoints
                        // so we need to make this an equality to prevent
                        // some of them possibly getting nudged apart.
                        thisSepDist = 0;
                        equality    = true;
                    }

                    Constraint* constraint = new Constraint(
                        prevVar,
                        vs[index],
                        thisSepDist,
                        equality
                    );
                    cs.push_back(constraint);
                    if (thisSepDist)
                    {
                        // Add to the list of gap constraints so we can
                        // rewrite the separation distance later.
                        gapcs.push_back(constraint);
                    }
                }
            }

            if (!currSegment->fixed)
            {
                // If this segment sees a channel boundary to its right,
                // then constrain its placement as such.
                if (currSegment->maxSpaceLimit < CHANNEL_MAX)
                {
                    vs.push_back(new Variable(
                        channelRightID,
                        currSegment->maxSpaceLimit,
                        fixedWeight
                    ));
                    cs.push_back(
                        new Constraint(vs[index], vs[vs.size() - 1], 0.0)
                    );
                }
            }

            prevVars.push_back(&(*currSegment));
        }

        std::list<PotentialSegmentConstraint> potentialConstraints;
        if (justUnifying)
        {
            for (std::list<size_t>::iterator curr = freeIndexes.begin();
                 curr != freeIndexes.end();
                 ++curr)
            {
                for (std::list<size_t>::iterator curr2 = curr;
                     curr2 != freeIndexes.end();
                     ++curr2)
                {
                    if (curr == curr2)
                    {
                        continue;
                    }
                    potentialConstraints.push_back(
                        PotentialSegmentConstraint(*curr, *curr2, vs)
                    );
                }
            }
        }
#ifdef NUDGE_DEBUG
        for (unsigned i = 0; i < vs.size(); ++i)
        {
            fprintf(stderr, "-vs[%d]=%f\n", i, vs[i]->desiredPosition);
        }
#endif
        // Repeatedly try solving this.  There are two cases:
        //  -  When Unifying, we greedily place as many free segments as
        //     possible at the same positions, that way they have more
        //     accurate nudging orders determined for them in the Nudging
        //     stage.
        //  -  When Nudging, if we can't fit all the segments with the
        //     default nudging distance we try smaller separation
        //     distances till we find a solution that is satisfied.
        bool justAddedConstraint = false;
        bool satisfied;

        typedef std::pair<size_t, size_t> UnsatisfiedRange;
        std::list<UnsatisfiedRange>       unsatisfiedRanges;
        do {
            IncSolver f(vs, cs);
            f.solve();

            // Determine if the problem was satisfied.
            satisfied = true;
            for (size_t i = 0; i < vs.size(); ++i)
            {
                // For each variable...
                if (vs[i]->id != freeSegmentID)
                {
                    // If it is a fixed segment (should stay still)...
                    if (fabs(vs[i]->finalPosition - vs[i]->desiredPosition)
                        > 0.0001)
                    {
                        // and it is not at it's desired position, then
                        // we consider the problem to be unsatisfied.
                        satisfied = false;

                        // We record ranges of unsatisfied variables based on
                        // the channel edges.
                        if (vs[i]->id == channelLeftID)
                        {
                            // This is the left-hand-side of a channel.
                            if (unsatisfiedRanges.empty()
                                || (unsatisfiedRanges.back().first
                                    != unsatisfiedRanges.back().second))
                            {
                                // There are no existing unsatisfied ranges,
                                // or there are but they are a valid range
                                // (we've encountered the right-hand channel
                                // edges already).
                                // So, start a new unsatisfied range.
                                unsatisfiedRanges.push_back(
                                    std::make_pair(i, i + 1)
                                );
                            }
                        }
                        else if (vs[i]->id == channelRightID)
                        {
                            // This is the right-hand-side of a channel.
                            if (unsatisfiedRanges.empty())
                            {
                                // There are no existing unsatisfied ranges,
                                // so start a new unsatisfied range.
                                // We are looking at a unsatisfied right side
                                // where the left side was satisfied, so the
                                // range begins at the previous variable
                                // which should be a left channel side.
                                COLA_ASSERT(i > 0);
                                COLA_ASSERT(vs[i - 1]->id == channelLeftID);
                                unsatisfiedRanges.push_back(
                                    std::make_pair(i - 1, i)
                                );
                            }
                            else
                            {
                                // Expand the existing range to include index.
                                unsatisfiedRanges.back().second = i;
                            }
                        }
                        else if (vs[i]->id == fixedSegmentID)
                        {
                            // Fixed connector segments can also start and
                            // extend unsatisfied variable ranges.
                            if (unsatisfiedRanges.empty())
                            {
                                // There are no existing unsatisfied ranges,
                                // so start a new unsatisfied range.
                                unsatisfiedRanges.push_back(std::make_pair(i, i)
                                );
                            }
                            else
                            {
                                // Expand the existing range to include index.
                                unsatisfiedRanges.back().second = i;
                            }
                        }
                    }
                }
            }

#ifdef NUDGE_DEBUG
            if (!satisfied)
            {
                fprintf(stderr, "unsatisfied\n");
            }
#endif

            if (justUnifying)
            {
                // When we're centring, we'd like to greedily place as many
                // segments as possible at the same positions, that way they
                // have more accurate nudging orders determined for them.
                //
                // We do this by taking pairs of adjoining free segments and
                // attempting to constrain them to have the same position,
                // starting from the closest up to the furthest.

                if (justAddedConstraint)
                {
                    COLA_ASSERT(potentialConstraints.size() > 0);
                    if (!satisfied)
                    {
                        // We couldn't satisfy the problem with the added
                        // potential constraint, so we can't position these
                        // segments together.  Roll back.
                        potentialConstraints.pop_front();
                        delete cs.back();
                        cs.pop_back();
                    }
                    else
                    {
                        // We could position these two segments together.
                        PotentialSegmentConstraint& pc
                            = potentialConstraints.front();

                        // Rewrite the indexes of these two variables to
                        // one, so we need not worry about redundant
                        // equality constraints.
                        for (std::list<PotentialSegmentConstraint>::iterator it
                             = potentialConstraints.begin();
                             it != potentialConstraints.end();
                             ++it)
                        {
                            it->rewriteIndex(pc.index1, pc.index2);
                        }
                        potentialConstraints.pop_front();
                    }
                }
                potentialConstraints.sort();
                justAddedConstraint = false;

                // Remove now invalid potential segment constraints.
                // This could have been caused by the variable rewriting.
                while (!potentialConstraints.empty()
                       && !potentialConstraints.front().stillValid())
                {
                    potentialConstraints.pop_front();
                }

                if (!potentialConstraints.empty())
                {
                    // We still have more possibilities to consider.
                    // Create a constraint for this, add it, and mark as
                    // unsatisfied, so the problem gets re-solved.
                    PotentialSegmentConstraint& pc
                        = potentialConstraints.front();
                    COLA_ASSERT(pc.index1 != pc.index2);
                    cs.push_back(
                        new Constraint(vs[pc.index1], vs[pc.index2], 0, true)
                    );
                    satisfied           = false;
                    justAddedConstraint = true;
                }
            }
            else
            {
                if (!satisfied)
                {
                    COLA_ASSERT(unsatisfiedRanges.size() > 0);
                    // Reduce the separation distance.
                    sepDist -= (baseSepDist / reductionSteps);
#ifndef NDEBUG
                    for (std::list<UnsatisfiedRange>::iterator it
                         = unsatisfiedRanges.begin();
                         it != unsatisfiedRanges.end();
                         ++it)
                    {
                        COLA_ASSERT(vs[it->first]->id != freeSegmentID);
                        COLA_ASSERT(vs[it->second]->id != freeSegmentID);
                    }
#endif
#ifdef NUDGE_DEBUG
                    for (std::list<UnsatisfiedRange>::iterator it
                         = unsatisfiedRanges.begin();
                         it != unsatisfiedRanges.end();
                         ++it)
                    {
                        fprintf(
                            stderr,
                            "unsatisfiedVarRange(%ld, %ld)\n",
                            it->first,
                            it->second
                        );
                    }
                    fprintf(stderr, "unsatisfied, trying %g\n", sepDist);
#endif
                    // And rewrite all the gap constraints to have the new
                    // reduced separation distance.
                    bool withinUnsatisfiedGroup = false;
                    for (Constraints::iterator cIt = cs.begin();
                         cIt != cs.end();
                         ++cIt)
                    {
                        UnsatisfiedRange& range = unsatisfiedRanges.front();
                        Constraint*       constraint = *cIt;

                        if (constraint->left == vs[range.first])
                        {
                            // Entered an unsatisfied range of variables.
                            withinUnsatisfiedGroup = true;
                        }

                        if (withinUnsatisfiedGroup && (constraint->gap > 0))
                        {
                            // Rewrite constraints in unsatisfied ranges
                            // that have a non-zero gap.
                            constraint->gap = sepDist;
                        }

                        if (constraint->right == vs[range.second])
                        {
                            // Left an unsatisfied range of variables.
                            withinUnsatisfiedGroup = false;
                            unsatisfiedRanges.pop_front();
                            if (unsatisfiedRanges.empty())
                            {
                                // And there are no more unsatisfied variables.
                                break;
                            }
                        }
                    }
                }
            }
        }
        while (!satisfied && (sepDist > 0.0001));

        if (satisfied)
        {
#ifdef NUDGE_DEBUG
            fprintf(stderr, "satisfied at nudgeDist = %g\n", sepDist);
#endif
            for (ShiftSegmentList::iterator currSegment = currentRegion.begin();
                 currSegment != currentRegion.end();
                 ++currSegment)
            {
                NudgingShiftSegment* segment
                    = static_cast<NudgingShiftSegment*>(*currSegment);

                segment->updatePositionsFromSolver(justUnifying);
            }
        }
#ifdef NUDGE_DEBUG
        for (unsigned i = 0; i < vs.size(); i++)
        {
            fprintf(stderr, "+vs[%d]=%f\n", i, vs[i]->finalPosition);
        }
#endif
#ifdef NUDGE_DEBUG_SVG
        for (ShiftSegmentList::iterator currSegment = currentRegion.begin();
             currSegment != currentRegion.end();
             ++currSegment)
        {
            NudgingShiftSegment* segment
                = static_cast<NudgingShiftSegment*>(*currSegment);

            fprintf(
                stdout,
                "<line style=\"stroke: #00F;\" x1=\"%g\" "
                "y1=\"%g\" x2=\"%g\" y2=\"%g\" />\n",
                segment->lowPoint()[XDIM],
                segment->variable->finalPosition,
                segment->highPoint()[XDIM],
                segment->variable->finalPosition
            );
        }
#endif
        for_each(currentRegion.begin(), currentRegion.end(), delete_object());
        for_each(vs.begin(), vs.end(), delete_object());
        for_each(cs.begin(), cs.end(), delete_object());
    }
}

void ImproveOrthogonalRoutes::simplifyOrthogonalRoutes(void)
{
    // Simplify routes.
    for (ConnRefList::const_iterator curr = m_router->connRefs.begin();
         curr != m_router->connRefs.end();
         ++curr)
    {
        if ((*curr)->routingType() != ConnType_Orthogonal)
        {
            continue;
        }
        (*curr)->set_route((*curr)->displayRoute().simplify());
    }
}

// Populates m_point_orders and m_shared_path_connectors_with_common_endpoints.
void ImproveOrthogonalRoutes::buildOrthogonalNudgingOrderInfo(void)
{
    // Simplify routes.
    simplifyOrthogonalRoutes();

    int crossingsN = 0;

    bool buildSharedPathInfo = false;
    if (!m_router->routingOption(Avoid::nudgeSharedPathsWithCommonEndPoint)
        && m_shared_path_connectors_with_common_endpoints.empty())
    {
        // We're not going to nudge apart shared paths with common ends so we
        // will need to store information about this during the crossing
        // detection.
        buildSharedPathInfo = true;
    }

    // Make a vector of the ConnRefList, for convenience.
    ConnRefVector connRefs(
        m_router->connRefs.begin(),
        m_router->connRefs.end()
    );

    // Make a temporary copy of all the connector displayRoutes.
    RouteVector connRoutes(connRefs.size());
    for (size_t ind = 0; ind < connRefs.size(); ++ind)
    {
        connRoutes[ind] = connRefs[ind]->displayRoute();
    }

    // Do segment splitting.
    for (size_t ind1 = 0; ind1 < connRefs.size(); ++ind1)
    {
        ConnRef* conn = connRefs[ind1];
        if (conn->routingType() != ConnType_Orthogonal)
        {
            continue;
        }

        for (size_t ind2 = 0; ind2 < connRefs.size(); ++ind2)
        {
            if (ind1 == ind2)
            {
                continue;
            }

            ConnRef* conn2 = connRefs[ind2];
            if (conn2->routingType() != ConnType_Orthogonal)
            {
                continue;
            }

            Avoid::Polygon& route  = connRoutes[ind1];
            Avoid::Polygon& route2 = connRoutes[ind2];
            splitBranchingSegments(route2, true, route);
        }
    }

    for (size_t ind1 = 0; ind1 < connRefs.size(); ++ind1)
    {
        ConnRef* conn = connRefs[ind1];
        if (conn->routingType() != ConnType_Orthogonal)
        {
            continue;
        }

        for (size_t ind2 = ind1 + 1; ind2 < connRefs.size(); ++ind2)
        {
            ConnRef* conn2 = connRefs[ind2];
            if (conn2->routingType() != ConnType_Orthogonal)
            {
                continue;
            }

            Avoid::Polygon&    route         = connRoutes[ind1];
            Avoid::Polygon&    route2        = connRoutes[ind2];
            int                crossings     = 0;
            unsigned int       crossingFlags = 0;
            ConnectorCrossings cross(route2, true, route, conn2, conn);
            cross.pointOrders = &m_point_orders;
            for (size_t i = 1; i < route.size(); ++i)
            {
                const bool finalSegment = ((i + 1) == route.size());
                cross.countForSegment(i, finalSegment);

                crossings     += cross.crossingCount;
                crossingFlags |= cross.crossingFlags;
            }
            if (crossings > 0)
            {
                crossingsN += crossings;
            }

            if (buildSharedPathInfo
                && (crossingFlags & CROSSING_SHARES_PATH_AT_END))
            {
                // Record if these two connectors have a shared path with a
                // common end point.
                m_shared_path_connectors_with_common_endpoints.insert(
                    UnsignedPair(conn->id(), conn2->id())
                );
            }
        }
    }
}

}  // namespace avoid
