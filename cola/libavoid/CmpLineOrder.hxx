///
/// @file CmpLineOrder.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __CMP_LINE_ORDER_HXX_F01E22380E98__
#define __CMP_LINE_ORDER_HXX_F01E22380E98__

namespace avoid {

class CmpLineOrder
{
public:
    CmpLineOrder(PtOrderMap& ord, const size_t dim)
        : orders(ord)
        , dimension(dim)
    {
    }

    bool operator()(
        const ShiftSegment* lhsSuper,
        const ShiftSegment* rhsSuper,
        bool*               comparable = nullptr
    ) const
    {
        const NudgingShiftSegment* lhs
            = static_cast<const NudgingShiftSegment*>(lhsSuper);
        const NudgingShiftSegment* rhs
            = static_cast<const NudgingShiftSegment*>(rhsSuper);
        if (comparable)
        {
            *comparable = true;
        }
        Point  lhsLow = lhs->lowPoint();
        Point  rhsLow = rhs->lowPoint();
        size_t altDim = (dimension + 1) % 2;
#ifndef NDEBUG
        const Point& lhsHigh = lhs->highPoint();
        const Point& rhsHigh = rhs->highPoint();
        COLA_ASSERT(lhsLow[dimension] == lhsHigh[dimension]);
        COLA_ASSERT(rhsLow[dimension] == rhsHigh[dimension]);
#endif

        // We consider things at effectively the same position to
        // be ordered based on their order and fixedOrder, so only
        // compare segments further apart than the nudgeDistance.
        if (lhsLow[dimension] != rhsLow[dimension])
        {
            return lhsLow[dimension] < rhsLow[dimension];
        }

        // If one of these is fixed, then determine order based on
        // fixed segment, that is, order so the fixed segment doesn't
        // block movement.
        bool      oneIsFixed    = false;
        const int lhsFixedOrder = lhs->fixedOrder(oneIsFixed);
        const int rhsFixedOrder = rhs->fixedOrder(oneIsFixed);
        if (oneIsFixed && (lhsFixedOrder != rhsFixedOrder))
        {
            return lhsFixedOrder < rhsFixedOrder;
        }

        // C-bends that did not have a clear order with s-bends might
        // not have a good ordering here, so compare their order in
        // terms of C-bend direction and S-bends and use that if it
        // differs for the two segments.
        const int lhsOrder = lhs->order();
        const int rhsOrder = rhs->order();
        if (lhsOrder != rhsOrder)
        {
            return lhsOrder < rhsOrder;
        }

        // Need to index using the original point into the map, so find it.
        Point& unchanged = (lhsLow[altDim] > rhsLow[altDim]) ? lhsLow : rhsLow;

        PtOrder& lowOrder = orders[unchanged];
        int      lhsPos   = lowOrder.positionFor(dimension, lhs->connRef);
        int      rhsPos   = lowOrder.positionFor(dimension, rhs->connRef);
        if ((lhsPos == -1) || (rhsPos == -1))
        {
            // A value for rhsPos or lhsPos mean the points are not directly
            // comparable, meaning they are at the same position but cannot
            // overlap (they are just collinear.  The relative order for
            // these segments is not important since we do not constrain
            // them against each other.
            // COLA_ASSERT(lhs->overlapsWith(rhs, dimension) == false);
            // We do need to be consistent though.
            if (comparable)
            {
                *comparable = false;
            }
            return lhsLow[altDim] < rhsLow[altDim];
        }
        return lhsPos < rhsPos;
    }

    PtOrderMap&  orders;
    const size_t dimension;
};

}  // namespace avoid

#endif  // __CMP_LINE_ORDER_HXX_F01E22380E98__
