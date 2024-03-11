///
/// @file LineSegment.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __LINE_SEGMENT_HXX_5453193718FF__
#define __LINE_SEGMENT_HXX_5453193718FF__

#include <list>

namespace avoid {

// Temporary structure used to store the possible horizontal visibility
// lines arising from the vertical sweep.
class LineSegment
{
public:
    LineSegment(
        const double& b,
        const double& f,
        const double& p,
        bool          ss  = false,
        VertInf*      bvi = nullptr,
        VertInf*      fvi = nullptr
    )
        : begin(b)
        , finish(f)
        , pos(p)
        , shapeSide(ss)
    {
        COLA_ASSERT(begin < finish);

        if (bvi)
        {
            vertInfs.insert(bvi);
        }
        if (fvi)
        {
            vertInfs.insert(fvi);
        }
    }

    LineSegment(const double& bf, const double& p, VertInf* bfvi = nullptr)
        : begin(bf)
        , finish(bf)
        , pos(p)
        , shapeSide(false)
    {
        if (bfvi)
        {
            vertInfs.insert(bfvi);
        }
    }

    // Order by begin, pos, finish.
    bool operator<(const LineSegment& rhs) const
    {
        if (begin != rhs.begin)
        {
            return begin < rhs.begin;
        }
        if (pos != rhs.pos)
        {
            return pos < rhs.pos;
        }
        if (finish != rhs.finish)
        {
            return finish < rhs.finish;
        }
        COLA_ASSERT(shapeSide == rhs.shapeSide);
        return false;
    }

    bool overlaps(const LineSegment& rhs) const
    {
        if ((begin == rhs.begin) && (pos == rhs.pos) && (finish == rhs.finish))
        {
            // Lines are exactly equal.
            return true;
        }

        if (pos == rhs.pos)
        {
            if (((begin >= rhs.begin) && (begin <= rhs.finish))
                || ((rhs.begin >= begin) && (rhs.begin <= finish)))
            {
                // They are colinear and overlap by some amount.
                return true;
            }
        }
        return false;
    }

    void mergeVertInfs(const LineSegment& segment)
    {
        begin  = std::min(begin, segment.begin);
        finish = std::max(finish, segment.finish);
        vertInfs.insert(segment.vertInfs.begin(), segment.vertInfs.end());
    }

    VertInf* beginVertInf(void) const
    {
        if (vertInfs.empty())
        {
            return nullptr;
        }
        VertInf* inf = *vertInfs.begin();
        if (((inf->point.y == begin) && (inf->point.x == pos))
            || ((inf->point.x == begin) && (inf->point.y == pos)))
        {
            // Only return the point if it is actually at the begin pos.
            return inf;
        }
        return nullptr;
    }

    VertInf* finishVertInf(void) const
    {
        if (vertInfs.empty())
        {
            return nullptr;
        }
        VertInf* inf = *vertInfs.rbegin();
        if (((inf->point.y == finish) && (inf->point.x == pos))
            || ((inf->point.x == finish) && (inf->point.y == pos)))
        {
            // Only return the point if it is actually at the finish pos.
            return inf;
        }
        return nullptr;
    }

    VertInf* commitPositionX(Router* router, double posX)
    {
        VertInf* found = nullptr;
        for (VertSet::iterator v = vertInfs.begin(); v != vertInfs.end(); ++v)
        {
            if ((*v)->point.x == posX)
            {
                found = *v;
                break;
            }
        }
        if (!found)
        {
            found = new VertInf(router, dummyOrthogID, Point(posX, pos));
            vertInfs.insert(found);
        }
        return found;
    }

    // Set begin endpoint vertex if none has been assigned.
    void horiCommitBegin(Router* router, VertInf* vert = nullptr)
    {
        if (vert)
        {
            vertInfs.insert(vert);
        }

        if (vertInfs.empty() || ((*vertInfs.begin())->point.x != begin))
        {
            if (begin != -DBL_MAX)
            {
                vertInfs.insert(
                    new VertInf(router, dummyOrthogID, Point(begin, pos))
                );
            }
        }
    }

    // Set begin endpoint vertex if none has been assigned.
    void horiCommitFinish(Router* router, VertInf* vert = nullptr)
    {
        if (vert)
        {
            vertInfs.insert(vert);
        }

        if (vertInfs.empty() || ((*vertInfs.rbegin())->point.x != finish))
        {
            if (finish != DBL_MAX)
            {
                vertInfs.insert(
                    new VertInf(router, dummyOrthogID, Point(finish, pos))
                );
            }
        }
    }

    // Converts a section of the points list to a set of breakPoints.
    // Returns the first of the intersection points occurring at finishPos.
    VertSet::iterator addSegmentsUpTo(double finishPos)
    {
        VertSet::iterator firstIntersectionPt = vertInfs.end();
        for (VertSet::iterator vert = vertInfs.begin(); vert != vertInfs.end();
             ++vert)
        {
            if ((*vert)->point.x > finishPos)
            {
                // We're done.
                break;
            }

            breakPoints.insert(PosVertInf(
                (*vert)->point.x,
                (*vert),
                getPosVertInfDirections(*vert, XDIM)
            ));

            if ((firstIntersectionPt == vertInfs.end())
                && ((*vert)->point.x == finishPos))
            {
                firstIntersectionPt = vert;
            }
        }
        // Returns the first of the intersection points at finishPos.
        return firstIntersectionPt;
    }

    // Add visibility edge(s) for this segment.  There may be multiple if
    // one of the endpoints is shared by multiple connector endpoints.
    void addEdgeHorizontal(Router* router)
    {
        horiCommitBegin(router);
        horiCommitFinish(router);

        addSegmentsUpTo(finish);
    }

    // Set flags to show what can be passed on this visibility line.
    // This can be used later to disregard some edges in the visibility
    // graph when routing particular connectors.
    void setLongRangeVisibilityFlags(size_t dim)
    {
        // First, travel in one direction
        bool seenConnPt    = false;
        bool seenShapeEdge = false;
        for (BreakpointSet::iterator nvert = breakPoints.begin();
             nvert != breakPoints.end();
             ++nvert)
        {
            VertIDProps mask = 0;
            if (dim == XDIM)
            {
                if (seenConnPt)
                {
                    mask |= XL_CONN;
                }
                if (seenShapeEdge)
                {
                    mask |= XL_EDGE;
                }
            }
            else  // if (dim == YDIM)
            {
                if (seenConnPt)
                {
                    mask |= YL_CONN;
                }
                if (seenShapeEdge)
                {
                    mask |= YL_EDGE;
                }
            }
            nvert->vert->orthogVisPropFlags |= mask;

            if (nvert->vert->id.isConnPt())
            {
                seenConnPt = true;
            }
            if (nvert->vert->id.isOrthShapeEdge())
            {
                seenShapeEdge = true;
            }
        }
        // Then in the other direction
        seenConnPt    = false;
        seenShapeEdge = false;
        for (BreakpointSet::reverse_iterator rvert = breakPoints.rbegin();
             rvert != breakPoints.rend();
             ++rvert)
        {
            VertIDProps mask = 0;
            if (dim == XDIM)
            {
                if (seenConnPt)
                {
                    mask |= XH_CONN;
                }
                if (seenShapeEdge)
                {
                    mask |= XH_EDGE;
                }
            }
            else  // if (dim == YDIM)
            {
                if (seenConnPt)
                {
                    mask |= YH_CONN;
                }
                if (seenShapeEdge)
                {
                    mask |= YH_EDGE;
                }
            }
            rvert->vert->orthogVisPropFlags |= mask;

            if (rvert->vert->id.isConnPt())
            {
                seenConnPt = true;
            }
            if (rvert->vert->id.isOrthShapeEdge())
            {
                seenShapeEdge = true;
            }
        }
    }

    // Add visibility edge(s) for this segment up until an intersection.
    // Then, move the segment beginning to the intersection point, so we
    // later only consider the remainder of the segment.
    // There may be multiple segments added to the graph if the beginning
    // endpoint of the segment is shared by multiple connector endpoints.
    VertSet addEdgeHorizontalTillIntersection(
        Router*      router,
        LineSegment& vertLine
    )
    {
        VertSet intersectionSet;

        horiCommitBegin(router);

        // Does a vertex already exist for this point.
        commitPositionX(router, vertLine.pos);

        // Generate segments and set end iterator to the first point
        // at the intersection position.
        VertSet::iterator restBegin = addSegmentsUpTo(vertLine.pos);

        // Add the intersections points to intersectionSet.
        VertSet::iterator restEnd = restBegin;
        while ((restEnd != vertInfs.end())
               && (*restEnd)->point.x == vertLine.pos)
        {
            ++restEnd;
        }
        intersectionSet.insert(restBegin, restEnd);

        // Adjust segment to remove processed portion.
        begin = vertLine.pos;
        vertInfs.erase(vertInfs.begin(), restBegin);

        return intersectionSet;
    }

    // Insert vertical breakpoints.
    void insertBreakpointsBegin(Router* router, LineSegment& vertLine)
    {
        VertInf* vert = nullptr;
        if (pos == vertLine.begin && vertLine.beginVertInf())
        {
            vert = vertLine.beginVertInf();
        }
        else if (pos == vertLine.finish && vertLine.finishVertInf())
        {
            vert = vertLine.finishVertInf();
        }
        horiCommitBegin(router, vert);

        for (VertSet::iterator v = vertInfs.begin(); v != vertInfs.end(); ++v)
        {
            if ((*v)->point.x == begin)
            {
                vertLine.breakPoints.insert(
                    PosVertInf(pos, *v, getPosVertInfDirections(*v, YDIM))
                );
            }
        }
    }

    // Insert vertical breakpoints.
    void insertBreakpointsFinish(Router* router, LineSegment& vertLine)
    {
        VertInf* vert = nullptr;
        if (pos == vertLine.begin && vertLine.beginVertInf())
        {
            vert = vertLine.beginVertInf();
        }
        else if (pos == vertLine.finish && vertLine.finishVertInf())
        {
            vert = vertLine.finishVertInf();
        }
        horiCommitFinish(router, vert);

        for (VertSet::iterator v = vertInfs.begin(); v != vertInfs.end(); ++v)
        {
            if ((*v)->point.x == finish)
            {
                vertLine.breakPoints.insert(
                    PosVertInf(pos, *v, getPosVertInfDirections(*v, YDIM))
                );
            }
        }
    }

    void generateVisibilityEdgesFromBreakpointSet(Router* router, size_t dim)
    {
        if (breakPoints.empty() || ((breakPoints.begin())->pos > begin))
        {
            // Add a begin point if there was not already an intersection
            // found at that point. Though, don't do this if the line
            // segment goes off to infinity -- we can't reach anything
            // by going in that direction!
            if (begin == -DBL_MAX)
            {
                // Shorten line to first intersection point.
                COLA_ASSERT(!breakPoints.empty());
                begin = breakPoints.begin()->pos;
            }
            else
            {
                // Add begin point.
                Point point(pos, pos);
                point[dim]    = begin;
                VertInf* vert = new VertInf(router, dummyOrthogID, point);
                breakPoints.insert(PosVertInf(begin, vert));
            }
        }
        if (breakPoints.empty() || ((breakPoints.rbegin())->pos < finish))
        {
            // Add a finish point if there was not already an intersection
            // found at that point. Though, don't do this if the line
            // segment goes off to infinity -- we can't reach anything
            // by going in that direction!
            if (finish == DBL_MAX)
            {
                // Shorten line to last intersection point.
                finish = breakPoints.rbegin()->pos;
            }
            else
            {
                // Add finish point.
                Point point(pos, pos);
                point[dim]    = finish;
                VertInf* vert = new VertInf(router, dummyOrthogID, point);
                breakPoints.insert(PosVertInf(finish, vert));
            }
        }

        // Set flags for orthogonal routing optimisation.
        setLongRangeVisibilityFlags(dim);

        const bool              orthogonal = true;
        BreakpointSet::iterator vert, last;
#if 0
        last = breakPoints.end();
        for (vert = breakPoints.begin(); vert != breakPoints.end();)
        {
            if (vert->vert->id == dummyOrthogID)
            {
                if (last == breakPoints.end() ||
                        (last->vert->point != vert->vert->point))
                {
                    last = vert;
                }
                else
                {
                    // Already seen a dummy orthogonal point at this
                    // position, delete it.

            }
            else
            {
                ++vert;
            }
        }
#endif
        for (vert = last = breakPoints.begin(); vert != breakPoints.end();)
        {
            BreakpointSet::iterator firstPrev = last;
            while (last->vert->point[dim] != vert->vert->point[dim])
            {
                COLA_ASSERT(vert != last);
                // Assert points are not at the same position.
                COLA_ASSERT(vert->vert->point != last->vert->point);

                if (vert->vert->id.isConnPt() && last->vert->id.isConnPt())
                {
                    // Here we have a pair of two endpoints that are both
                    // connector endpoints and both are inside a shape.

                    // Give vert visibility back to the first non-connector
                    // endpoint vertex (i.e., the side of the shape).
                    BreakpointSet::iterator side = last;
                    while (side->vert->id.isConnPt())
                    {
                        if (side == breakPoints.begin())
                        {
                            break;
                        }
                        --side;
                    }
                    bool canSeeDown = (vert->dirs & VisDirDown);
                    if (canSeeDown && !(side->vert->id.isConnPt()))
                    {
                        EdgeInf* edge
                            = new EdgeInf(side->vert, vert->vert, orthogonal);
                        edge->setDist(
                            vert->vert->point[dim] - side->vert->point[dim]
                        );
                    }

                    // Give last visibility back to the first non-connector
                    // endpoint vertex (i.e., the side of the shape).
                    side = vert;
                    while ((side != breakPoints.end())
                           && side->vert->id.isConnPt())
                    {
                        ++side;
                    }
                    bool canSeeUp = (last->dirs & VisDirUp);
                    if (canSeeUp && (side != breakPoints.end()))
                    {
                        EdgeInf* edge
                            = new EdgeInf(last->vert, side->vert, orthogonal);
                        edge->setDist(
                            side->vert->point[dim] - last->vert->point[dim]
                        );
                    }
                }

                // The normal case.
                //
                // Note: It's okay to give two connector endpoints visibility
                // here since we only consider the partner endpoint as a
                // candidate while searching if it is the other endpoint of
                // the connector in question.
                //
                bool generateEdge = true;
                if (last->vert->id.isConnPt() && !(last->dirs & VisDirUp))
                {
                    // Don't generate the visibility segment if the ConnEnd
                    // doesn't have visibility in that direction.
                    generateEdge = false;
                }
                else if (vert->vert->id.isConnPt() && !(vert->dirs & VisDirDown))
                {
                    // Don't generate the visibility segment if the ConnEnd
                    // doesn't have visibility in that direction.
                    generateEdge = false;
                }
                if (generateEdge)
                {
                    EdgeInf* edge
                        = new EdgeInf(last->vert, vert->vert, orthogonal);
                    edge->setDist(
                        vert->vert->point[dim] - last->vert->point[dim]
                    );
                }

                ++last;
            }

            ++vert;

            if ((vert != breakPoints.end())
                && (last->vert->point[dim] == vert->vert->point[dim]))
            {
                // Still looking at same pair, just reset prev number pointer.
                last = firstPrev;
            }
            else
            {
                // vert has moved to the beginning of a group at a new
                // position.  Last is now in the right place, so do nothing.
            }
        }
    }

    double begin;
    double finish;
    double pos;

    // XXX shapeSide is unused and could possibly be removed?
    bool shapeSide;

    VertSet       vertInfs;
    BreakpointSet breakPoints;

private:
    // MSVC wants to generate the assignment operator and the default
    // constructor, but fails.  Therefore we declare them private and
    // don't implement them.
    LineSegment& operator=(const LineSegment&);
    LineSegment();
};

using SegmentList = std::list<LineSegment>;

}  // namespace avoid

#endif  // __LINE_SEGMENT_HXX_5453193718FF__
