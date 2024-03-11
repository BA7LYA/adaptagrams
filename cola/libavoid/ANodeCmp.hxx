///
/// @file ANodeCmp.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __A_NODE_CMP_HXX_971A8AEB5A7C__
#define __A_NODE_CMP_HXX_971A8AEB5A7C__

#include <cmath>

#include "libavoid/ANode.hxx"

namespace avoid {

// This returns the opposite result (>) so that when used with stl::make_heap,
// the head node of the heap will be the smallest value, rather than the
// largest.  This saves us from having to sort the heap (and then reorder
// it back into a heap) when getting the next node to examine.  This way we
// get better complexity -- logarithmic pushes and pops to the heap.
//
class ANodeCmp
{
public:
    ANodeCmp() {}

    bool operator()(const ANode* a, const ANode* b)
    {
        // We need to use an epsilon here since otherwise the multiple addition
        // of floating point numbers that makes up the 'f' values cause a
        // problem with routings occasionally being non-deterministic.
        if (fabs(a->f - b->f) > 0.0000001)
        {
            return a->f > b->f;
        }
        if (a->timeStamp != b->timeStamp)
        {
            // Tiebreaker, if two paths have equal cost, then choose the one
            // with the highest timeStamp.  This corresponds to the furthest
            // point explored along the straight-line path.  When exploring we
            // give the directions the following timeStamps; left:1, right:2 and
            // forward:3, then we always try to explore forward first.
            return a->timeStamp < b->timeStamp;
        }
        return false;
    }
};

}  // namespace avoid

#endif  // __A_NODE_CMP_HXX_971A8AEB5A7C__
