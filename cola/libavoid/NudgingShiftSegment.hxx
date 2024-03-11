///
/// @file NudgingShiftSegment.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __NUDGING_SHIFT_SEGMENT_HXX_E5CF674BDA8F__
#define __NUDGING_SHIFT_SEGMENT_HXX_E5CF674BDA8F__

#include "libavoid/ShiftSegment.hxx"

namespace avoid {

class NudgingShiftSegment : public ShiftSegment
{
public:
    // For shiftable segments.
    NudgingShiftSegment(
        ConnRef*     conn,
        const size_t low,
        const size_t high,
        bool         isSBend,
        bool         isZBend,
        const size_t dim,
        double       minLim,
        double       maxLim
    )
        : ShiftSegment(dim)
        , connRef(conn)
        , variable(nullptr)
        , fixed(false)
        , finalSegment(false)
        , endsInShape(false)
        , singleConnectedSegment(false)
        , sBend(isSBend)
        , zBend(isZBend)
    {
        indexes.push_back(low);
        indexes.push_back(high);
        minSpaceLimit = minLim;
        maxSpaceLimit = maxLim;
    }

    // For fixed segments.
    NudgingShiftSegment(
        ConnRef*     conn,
        const size_t low,
        const size_t high,
        const size_t dim
    )
        : ShiftSegment(dim)
        , connRef(conn)
        , variable(nullptr)
        , fixed(true)
        , finalSegment(false)
        , endsInShape(false)
        , singleConnectedSegment(false)
        , sBend(false)
        , zBend(false)
    {
        indexes.push_back(low);
        indexes.push_back(high);
        // This has no space to shift.
        minSpaceLimit = lowPoint()[dim];
        maxSpaceLimit = lowPoint()[dim];
    }

    virtual ~NudgingShiftSegment() {}

    Point& lowPoint(void)
    {
        return connRef->displayRoute().ps[indexes.front()];
    }

    Point& highPoint(void)
    {
        return connRef->displayRoute().ps[indexes.back()];
    }

    const Point& lowPoint(void) const
    {
        return connRef->displayRoute().ps[indexes.front()];
    }

    const Point& highPoint(void) const
    {
        return connRef->displayRoute().ps[indexes.back()];
    }

    double nudgeDistance(void) const
    {
        return connRef->router()->routingParameter(idealNudgingDistance);
    }

    bool immovable(void) const
    {
        return !zigzag();
    }

    void createSolverVariable(const bool justUnifying)
    {
        bool nudgeFinalSegments = connRef->router()->routingOption(
            nudgeOrthogonalSegmentsConnectedToShapes
        );
        int    varID  = freeSegmentID;
        double varPos = lowPoint()[dimension];
        double weight = freeWeight;
        if (nudgeFinalSegments && finalSegment)
        {
            weight = strongWeight;

            if (singleConnectedSegment && !justUnifying)
            {
                // This is a single segment connector bridging
                // two shapes.  So, we want to try to keep it
                // centred rather than shift it.
                // Don't do this during Unifying stage, or else
                // these connectors could end up at slightly
                // different positions and get the wrong ordering
                // for nudging.
                weight = strongerWeight;
            }
        }
        else if (checkpoints.size() > 0)
        {
            weight = strongWeight;
        }
        else if (zigzag())
        {
            COLA_ASSERT(minSpaceLimit > -CHANNEL_MAX);
            COLA_ASSERT(maxSpaceLimit < CHANNEL_MAX);

            // For zigzag bends, take the middle as ideal.
            varPos = minSpaceLimit + ((maxSpaceLimit - minSpaceLimit) / 2);
        }
        else if (fixed)
        {
            // Fixed segments shouldn't get moved.
            weight = fixedWeight;
            varID  = fixedSegmentID;
        }
        else if (!finalSegment)
        {
            // Set a higher weight for c-bends to stop them sometimes
            // getting pushed out into channels by more-free connectors
            // to the "inner" side of them.
            weight = strongWeight;
        }

        variable = new Variable(varID, varPos, weight);
    }

    void updatePositionsFromSolver(const bool justUnifying)
    {
        if (fixed)
        {
            return;
        }
        double newPos = variable->finalPosition;

        // The solver can sometimes cause variables to be outside their
        // limits by a tiny amount, since all variables are held by
        // weights.  Thus, just make sure they stay in their limits.
        newPos = std::max(newPos, minSpaceLimit);
        newPos = std::min(newPos, maxSpaceLimit);

#ifdef NUDGE_DEBUG
        printf("Pos: %lX, %.16f\n", (long)connRef, newPos);
#endif
        for (size_t it = 0; it < indexes.size(); ++it)
        {
            size_t index                                 = indexes[it];
            connRef->displayRoute().ps[index][dimension] = newPos;
        }

#ifdef DEBUGHANDLER
        if (!justUnifying && connRef->router()->debugHandler())
        {
            connRef->router()->debugHandler()->updateConnectorRoute(
                connRef,
                indexes[0],
                indexes[indexes.size() - 1]
            );
        }
#endif
    }

    int fixedOrder(bool& isFixed) const
    {
        double nudgeDist  = nudgeDistance();
        double pos        = lowPoint()[dimension];
        bool   minLimited = ((pos - minSpaceLimit) < nudgeDist);
        bool   maxLimited = ((maxSpaceLimit - pos) < nudgeDist);

        if (fixed || (minLimited && maxLimited))
        {
            isFixed = true;
            return 0;
        }
        else if (minLimited)
        {
            return 1;
        }
        else if (maxLimited)
        {
            return -1;
        }
        return 0;
    }

    int order(void) const
    {
        if (lowC())
        {
            return -1;
        }
        else if (highC())
        {
            return 1;
        }
        return 0;
    }

    bool zigzag(void) const
    {
        return sBend || zBend;
    }

    // This counts segments that are collinear and share an endpoint as
    // overlapping.  This allows them to be nudged apart where possible.
    bool overlapsWith(const ShiftSegment* rhsSuper, const size_t dim) const
    {
        const NudgingShiftSegment* rhs
            = static_cast<const NudgingShiftSegment*>(rhsSuper);
        size_t       altDim    = (dim + 1) % 2;
        const Point& lowPt     = lowPoint();
        const Point& highPt    = highPoint();
        const Point& rhsLowPt  = rhs->lowPoint();
        const Point& rhsHighPt = rhs->highPoint();
        if ((lowPt[altDim] < rhsHighPt[altDim])
            && (rhsLowPt[altDim] < highPt[altDim]))
        {
            // The segments overlap.
            if ((minSpaceLimit <= rhs->maxSpaceLimit)
                && (rhs->minSpaceLimit <= maxSpaceLimit))
            {
                return true;
            }
        }
        else if ((lowPt[altDim] == rhsHighPt[altDim]) || (rhsLowPt[altDim] == highPt[altDim]))
        {
            bool nudgeColinearSegments = connRef->router()->routingOption(
                nudgeOrthogonalTouchingColinearSegments
            );

            if ((minSpaceLimit <= rhs->maxSpaceLimit)
                && (rhs->minSpaceLimit <= maxSpaceLimit))
            {
                // The segments could touch at one end.
                if (connRef->router()->routingParameter(fixedSharedPathPenalty)
                    > 0)
                {
                    // If we are routing with a fixedSharedPathPenalty
                    // then we don't want these segments to ever touch
                    // or slide past each other, so they are always
                    // considered to be overlapping.
                    return true;
                }
                else if ((rhs->sBend && sBend) || (rhs->zBend && zBend))
                {
                    // Count them as overlapping for nudging if they
                    // are both s-bends or both z-bends, i.e., when
                    // the ordering would matter.
                    return nudgeColinearSegments;
                }
                else if ((rhs->finalSegment && finalSegment) && (rhs->connRef == connRef))
                {
                    return nudgeColinearSegments;
                }
            }
        }
        return false;
    }

    // These segments are allowed to drift into alignment but don't have to.
    bool canAlignWith(const NudgingShiftSegment* rhs, const size_t dim) const
    {
        COLA_UNUSED(dim);

        if (connRef != rhs->connRef)
        {
            return false;
        }

        // Don't allow segments of the same connector to drift together
        // where one of them goes via a checkpoint.  We want the path
        // through the checkpoint to be maintained.
        bool hasCheckpoints    = checkpoints.size() > 0;
        bool rhsHasCheckpoints = rhs->checkpoints.size() > 0;
        if (hasCheckpoints || rhsHasCheckpoints)
        {
            return false;
        }
        return true;
    }

    // These segments should align with each other.
    bool shouldAlignWith(const ShiftSegment* rhsSuper, const size_t dim) const
    {
        const NudgingShiftSegment* rhs
            = static_cast<const NudgingShiftSegment*>(rhsSuper);
        if ((connRef == rhs->connRef) && finalSegment && rhs->finalSegment
            && overlapsWith(rhs, dim))
        {
            // If both the segments are in shapes then we know limits
            // and can align.  Otherwise we do this just for segments
            // that are very close together, since these will often
            // prevent nudging, or force it to have a tiny separation
            // value.
            if ((endsInShape && rhs->endsInShape)
                || (fabs(lowPoint()[dim] - rhs->lowPoint()[dim]) < 10))
            {
                return true;
            }
        }
        else if ((connRef == rhs->connRef) &&
                     // Not both final
                     ((finalSegment & rhs->finalSegment) != true))
        {
            bool hasCheckpoints    = checkpoints.size() > 0;
            bool rhsHasCheckpoints = rhs->checkpoints.size() > 0;

            if (hasCheckpoints != rhsHasCheckpoints)
            {
                // At least one segment has checkpoints, but not both.

                size_t altDim = (dim + 1) % 2;
                double space  = fabs(lowPoint()[dim] - rhs->lowPoint()[dim]);
                double touchPos;
                bool   couldTouch = false;
                if (lowPoint()[altDim] == rhs->highPoint()[altDim])
                {
                    couldTouch = true;
                    touchPos   = lowPoint()[altDim];
                }
                else if (highPoint()[altDim] == rhs->lowPoint()[altDim])
                {
                    couldTouch = true;
                    touchPos   = highPoint()[altDim];
                }

                // We should align these so long as they are close
                // together (<= 10) and there isn't a checkpoint at the
                // touch point, i.e., we'd be altering the edges leading
                // into the checkpoint.  We want to keep these in place
                // and opportunistically move other edges to align with
                // them.
                return couldTouch && (space <= 10)
                    && !hasCheckpointAtPosition(touchPos, altDim)
                    && !rhs->hasCheckpointAtPosition(touchPos, altDim);
            }
        }
        return false;
    }

    // Used for merging segments with end segments that should appear as
    // a single segment.
    void mergeWith(const ShiftSegment* rhsSuper, const size_t dim)
    {
        // Adjust limits.
        minSpaceLimit = std::max(minSpaceLimit, rhsSuper->minSpaceLimit);
        maxSpaceLimit = std::min(maxSpaceLimit, rhsSuper->maxSpaceLimit);

        // Find a new position for the segment, taking into account
        // the two original positions and the combined limits.
        double segmentPos  = lowPoint()[dimension];
        double segment2Pos = rhsSuper->lowPoint()[dimension];
        if (segment2Pos < segmentPos)
        {
            segmentPos -= ((segmentPos - segment2Pos) / 2.0);
        }
        else if (segment2Pos > segmentPos)
        {
            segmentPos += ((segment2Pos - segmentPos) / 2.0);
        }
        segmentPos = std::max(minSpaceLimit, segmentPos);
        segmentPos = std::min(maxSpaceLimit, segmentPos);

        // Merge the index lists and sort the new list.
        const NudgingShiftSegment* rhs
            = static_cast<const NudgingShiftSegment*>(rhsSuper);
        indexes.insert(indexes.end(), rhs->indexes.begin(), rhs->indexes.end());
        size_t     altDim = (dim + 1) % 2;
        CmpIndexes compare(connRef, altDim);
        sort(indexes.begin(), indexes.end(), compare);

        // Apply the new positon to all points to keep them constant.
        for (size_t it = 0; it < indexes.size(); ++it)
        {
            size_t index                                 = indexes[it];
            connRef->displayRoute().ps[index][dimension] = segmentPos;
        }
    }

    bool hasCheckpointAtPosition(const double position, const size_t dim) const
    {
        for (size_t cp = 0; cp < checkpoints.size(); ++cp)
        {
            if (checkpoints[cp][dim] == position)
            {
                return true;
            }
        }
        return false;
    }

    ConnRef*            connRef;
    Variable*           variable;
    std::vector<size_t> indexes;
    bool                fixed;
    bool                finalSegment;
    bool                endsInShape;
    bool                singleConnectedSegment;
    std::vector<Point>  checkpoints;

private:
    bool sBend;
    bool zBend;

    bool lowC(void) const
    {
        // This is true if this is a cBend and its adjoining points
        // are at lower positions.
        if (!finalSegment && !zigzag() && !fixed
            && (minSpaceLimit == lowPoint()[dimension]))
        {
            return true;
        }
        return false;
    }

    bool highC(void) const
    {
        // This is true if this is a cBend and its adjoining points
        // are at higher positions.
        if (!finalSegment && !zigzag() && !fixed
            && (maxSpaceLimit == lowPoint()[dimension]))
        {
            return true;
        }
        return false;
    }
};

}  // namespace avoid

#endif  // __NUDGING_SHIFT_SEGMENT_HXX_E5CF674BDA8F__
