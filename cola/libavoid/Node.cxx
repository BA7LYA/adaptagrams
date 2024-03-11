///
/// @file Node.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/Node.hxx"

#include <algorithm>

#include "libavoid/Box.hxx"
#include "libavoid/Obstacle.hxx"
#include "libavoid/VertInf.hxx"

namespace avoid {

Node::Node(Obstacle* v, const double p)
    : v(v)
    , c(nullptr)
    , ss(nullptr)
    , pos(p)
    , firstAbove(nullptr)
    , firstBelow(nullptr)
{
    Box bBox  = v->routingBox();
    min[XDIM] = bBox.min.x;
    min[YDIM] = bBox.min.y;
    max[XDIM] = bBox.max.x;
    max[YDIM] = bBox.max.y;
    // COLA_ASSERT(r->width()<1e40);
}

Node::Node(VertInf* c, const double p)
    : v(nullptr)
    , c(c)
    , ss(nullptr)
    , pos(p)
    , firstAbove(nullptr)
    , firstBelow(nullptr)
{
    min[XDIM] = max[XDIM] = c->point.x;
    min[YDIM] = max[YDIM] = c->point.y;
}

Node::Node(ShiftSegment* ss, const double p)
    : v(nullptr)
    , c(nullptr)
    , ss(ss)
    , pos(p)
    , firstAbove(nullptr)
    , firstBelow(nullptr)
{
    // These values shouldn't ever be used, so they don't matter.
    min[XDIM] = max[XDIM] = min[YDIM] = max[YDIM] = 0;
}

Node::~Node() {}

// Find the first Node above in the scanline that is a shape edge,
// and does not have an open or close event at this position (meaning
// it is just about to be removed).
double Node::firstObstacleAbove(size_t dim)
{
    Node* curr = firstAbove;
    while (curr && (curr->ss || (curr->max[dim] > pos)))
    {
        curr = curr->firstAbove;
    }

    if (curr)
    {
        return curr->max[dim];
    }
    return -DBL_MAX;
}

// Find the first Node below in the scanline that is a shape edge,
// and does not have an open or close event at this position (meaning
// it is just about to be removed).
double Node::firstObstacleBelow(size_t dim)
{
    Node* curr = firstBelow;
    while (curr && (curr->ss || (curr->min[dim] < pos)))
    {
        curr = curr->firstBelow;
    }

    if (curr)
    {
        return curr->min[dim];
    }
    return DBL_MAX;
}

// Mark all connector segments above in the scanline as being able
// to see to this shape edge.
void Node::markShiftSegmentsAbove(size_t dim)
{
    Node* curr = firstAbove;
    while (curr && (curr->ss || (curr->pos > min[dim])))
    {
        if (curr->ss && (curr->pos <= min[dim]))
        {
            curr->ss->maxSpaceLimit
                = std::min(min[dim], curr->ss->maxSpaceLimit);
        }
        curr = curr->firstAbove;
    }
}

// Mark all connector segments below in the scanline as being able
// to see to this shape edge.
void Node::markShiftSegmentsBelow(size_t dim)
{
    Node* curr = firstBelow;
    while (curr && (curr->ss || (curr->pos < max[dim])))
    {
        if (curr->ss && (curr->pos >= max[dim]))
        {
            curr->ss->minSpaceLimit
                = std::max(max[dim], curr->ss->minSpaceLimit);
        }
        curr = curr->firstBelow;
    }
}

void Node::findFirstPointAboveAndBelow(
    const size_t dim,
    const double linePos,
    double&      firstAbovePos,
    double&      firstBelowPos,
    double&      lastAbovePos,
    double&      lastBelowPos
)
{
    firstAbovePos = -DBL_MAX;
    firstBelowPos = DBL_MAX;
    // We start looking left from the right side of the shape,
    // and vice versa.
    lastAbovePos  = max[dim];
    lastBelowPos  = min[dim];

    Node* curr            = nullptr;
    bool  eventsAtSamePos = false;
    for (int direction = 0; direction < 2; ++direction)
    {
        // Look for obstacles in one direction, then the other.
        curr = (direction == 0) ? firstAbove : firstBelow;

        while (curr)
        {
            // The events are at a shared beginning or end of a shape or
            // connection point.  Note, connection points have the same
            // min and max value in the !dim dimension.
            eventsAtSamePos
                = (((linePos == max[!dim]) && (linePos == curr->max[!dim]))
                   || ((linePos == min[!dim]) && (linePos == curr->min[!dim])));

            if (curr->max[dim] <= min[dim])
            {
                // Curr shape is completely to the left,
                // so add it's right side as a limit
                firstAbovePos = std::max(curr->max[dim], firstAbovePos);
            }
            else if (curr->min[dim] >= max[dim])
            {
                // Curr shape is completely to the right,
                // so add it's left side as a limit
                firstBelowPos = std::min(curr->min[dim], firstBelowPos);
            }
            else if (!eventsAtSamePos)
            {
                // Shapes that open or close at the same position do not
                // block visibility, so if they are not at same position
                // determine where they overlap.
                lastAbovePos = std::min(curr->min[dim], lastAbovePos);
                lastBelowPos = std::max(curr->max[dim], lastBelowPos);
            }
            curr = (direction == 0) ? curr->firstAbove : curr->firstBelow;
        }
    }
}

double Node::firstPointAbove(size_t dim)
{
    // We are looking for the first obstacle above this position,
    // though we ignore shape edges if this point is inline with
    // the edge of the obstacle.  That is, points have visibility
    // along the edge of shapes.
    size_t altDim = (dim + 1) % 2;
    double result = -DBL_MAX;
    Node*  curr   = firstAbove;
    while (curr)
    {
        bool inLineWithEdge = (min[altDim] == curr->min[altDim])
                           || (min[altDim] == curr->max[altDim]);
        if (!inLineWithEdge && (curr->max[dim] <= pos))
        {
            result = std::max(curr->max[dim], result);
        }
        curr = curr->firstAbove;
    }
    return result;
}

double Node::firstPointBelow(size_t dim)
{
    // We are looking for the first obstacle below this position,
    // though we ignore shape edges if this point is inline with
    // the edge of the obstacle.  That is, points have visibility
    // along the edge of shapes.
    size_t altDim = (dim + 1) % 2;
    double result = DBL_MAX;
    Node*  curr   = firstBelow;
    while (curr)
    {
        bool inLineWithEdge = (min[altDim] == curr->min[altDim])
                           || (min[altDim] == curr->max[altDim]);
        if (!inLineWithEdge && (curr->min[dim] >= pos))
        {
            result = std::min(curr->min[dim], result);
        }
        curr = curr->firstBelow;
    }
    return result;
}

// This is a bit inefficient, but we won't need to do it once we have
// connection points.
bool Node::isInsideShape(size_t dimension)
{
    for (Node* curr = firstBelow; curr; curr = curr->firstBelow)
    {
        if ((curr->min[dimension] < pos) && (pos < curr->max[dimension]))
        {
            return true;
        }
    }
    for (Node* curr = firstAbove; curr; curr = curr->firstAbove)
    {
        if ((curr->min[dimension] < pos) && (pos < curr->max[dimension]))
        {
            return true;
        }
    }
    return false;
}

}  // namespace avoid
