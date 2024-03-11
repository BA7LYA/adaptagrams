///
/// @file Segment.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include <functional>
#include <vector>

namespace topology {

class Edge;
class EdgePoint;
class Node;

/*
 * a Segment is one straightline segment between two EdgePoint which are
 * either bend points and/or ends of the edge.
 */
class Segment
{
public:
    /*
     * Create segment for a given edge between two EdgePoints.
     * Note that segments can be zero length, for example between
     * opposite corners of two rectangles.
     * @param edge the edge to which this segment belongs
     * @param start the EdgePoint at the start of the segment
     * @param end the EdgePoint at the end of the segment
     */
    Segment(Edge* edge, EdgePoint* start, EdgePoint* end)
        : edge(edge)
        , start(start)
        , end(end)
    {
        // no self loops!
        COLA_ASSERT(start != end);
        // the ends of the segment should not involve the same rectangle vertex
        COLA_ASSERT(!start->uniqueCheck(end));
        start->outSegment = this;
        end->inSegment    = this;
    }

    /*
     * add a new StraightConstraint to this segment (if necessary)
     * @param node the node with which the constraint is associated
     * @param pos the scanPos, i.e. the position in the scan dimension
     * of the opening or closing of node.
     * @return true if a constraint was created
     */
    bool createStraightConstraint(vpsc::Dim dim, Node* node, double pos);

    /*
     * creates a copy of the StraightConstraint in our own
     * straightConstraints list, but only if this segment is not directly
     * connected to the centre of the StraightConstraint node.  @param s
     * the StraightConstraint to be copied across
     */
    void transferStraightConstraint(StraightConstraint* s);

    /*
     * this typedef can be used to declare a wrapper functor
     * for transferStraightConstraint
     */
    using TransferStraightConstraint = std::function<void(StraightConstraint*)>;

    /*
     * TransferStraightConstraint might for example be applied to
     * forEachStraightConstraint
     */
    template<typename T>
    void forEachStraightConstraint(T f)
    {
        for_each(straightConstraints.begin(), straightConstraints.end(), f);
    }

    /*
     * append straightConstraints to ts
     */
    void getStraightConstraints(std::vector<TopologyConstraint*>* ts) const;

    /*
     * clean up topologyConstraints
     */
    void deleteStraightConstraints();

    ~Segment();

    // the edge which this segment is part of
    Edge* edge;

    // the start point of the segment - either the end of the edge
    // if connected to a real node, or a bend point
    EdgePoint* start;

    // the end point of the segment
    EdgePoint* end;

    /*
     * @return the EdgePoint at the minimum extent of this segment on the
     * scan axis
     */
    EdgePoint* getMin(vpsc::Dim scanDim) const
    {
        if (start->pos(vpsc::conjugate(scanDim))
            <= end->pos(vpsc::conjugate(scanDim)))
        {
            return start;
        }
        return end;
    }

    /*
     * @return the EdgePoint on the maximum extent of this segment on the
     * scan axis
     */
    EdgePoint* getMax(vpsc::Dim scanDim) const
    {
        if (start->pos(vpsc::conjugate(scanDim))
            > end->pos(vpsc::conjugate(scanDim)))
        {
            return start;
        }
        return end;
    }

    /*
     * compute the intersection with the line !dim=pos.
     * if called when Segment is parallel to scan line it will throw an
     * assertion error.
     * @param pos position of scanline
     * @param p distance along line from start to end at which intersection
     * occurs (where 0 is at the start and 1 is at the end -- though
     * note that p will be outside this range for BendConstraints).
     * @return position along scanline of intersection with the line along
     * this edge segment
     */
    double forwardIntersection(vpsc::Dim scanDim, double pos, double& p) const
    {
        return intersection(scanDim, pos, start, end, p);
    }

    double reverseIntersection(vpsc::Dim scanDim, double pos, double& p) const
    {
        return intersection(scanDim, pos, end, start, p);
    }

    double forwardIntersection(vpsc::Dim scanDim, double pos) const
    {
        double p;
        return forwardIntersection(scanDim, pos, p);
    }

    double intersection(
        vpsc::Dim        scanDim,
        const double     pos,
        const EdgePoint* s,
        const EdgePoint* e,
        double&          p
    ) const
    {
        double ux    = s->pos(scanDim);
        double vx    = e->pos(scanDim);
        double uy    = s->pos(vpsc::conjugate(scanDim));
        double vy    = e->pos(vpsc::conjugate(scanDim));
        double denom = vy - uy;
        COLA_ASSERT(denom != 0);  // must not be parallel to scanline!
        p = (pos - uy) / denom;
        return ux + p * (vx - ux);
    }

    std::string toString() const;

    /*
     * Compute the length in the specified dimension.
     */
    double length(vpsc::Dim dim) const;

    /*
     * Compute the euclidean distance between #start and #end.
     */
    double length() const;

    void assertNonZeroLength() const;

    /*
     * does this segment have Node v as a CENTRE start or end point?
     */
    bool connectedToNode(const Node* v) const;

private:
    // a set of topology constraints (left-/right-/above-/below-of
    // relationships / between this segment and nodes
    std::vector<StraightConstraint*> straightConstraints;
};

}  // namespace topology
