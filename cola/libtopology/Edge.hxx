///
/// @file Edge.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include <vector>

namespace straightener {
struct Route;
}  // namespace straightener

namespace topology {

class Segment;
class TopologyConstraint;

/**
 * @brief Topology representation of an edge.
 *
 * An edge provides a doubly linked list of segments, each involving a pair
 * of EdgePoints.
 *
 * @note You shouldn't need to create these yourself, but you may
 *       extract them from an existing ColaTopologyAddon and construct
 *       a new ColaTopologyAddon with that same topology information.
 */
class Edge
{
public:
    /// id specified by user.  Can be used to match to external edge.
    unsigned id;

    /// the ideal length which the layout should try to obtain for this edge
    double idealLength;

    /**
     * Head of a doubly-linked list of Segment each involving a pair of
     * EdgePoints
     */
    Segment* firstSegment;

    /**
     * End of list of Segment
     */
    Segment* lastSegment;

    // size of segments list headed by firstSegment
    size_t nSegments;

    /**
     * Construct an edge from a list of EdgePoint in sequence
     */
    Edge(unsigned id, double idealLength, EdgePoints& vs)
        : id(id)
        , idealLength(idealLength)
        , firstSegment(nullptr)
        , lastSegment(nullptr)
        , nSegments(0)
    {
        EdgePoints::iterator a = vs.begin();
        for (EdgePoints::iterator b = a + 1; b != vs.end(); ++a, ++b)
        {
            Segment* s = new Segment(this, *a, *b);
            nSegments++;
            if (firstSegment == nullptr)
            {
                firstSegment = s;
            }
            lastSegment = s;
        }
    }

    /*
     * apply an operation to every Segment and EdgePoint associated with
     * this Edge
     * @param po operation to apply to each EdgePoint
     * @param so operation to apply to each Segment
     */
    template<typename PointOp, typename SegmentOp>
    void forEach(PointOp po, SegmentOp so, bool noCycle = false)
    {
        ForEach<Edge*, PointOp, SegmentOp>(this, po, so, noCycle);
    }

    /*
     * apply an operation to every Segment and EdgePoint associated with
     * this Edge, without changing anything
     * @param po operation to apply to each EdgePoint
     * @param so operation to apply to each Segment
     */
    template<typename PointOp, typename SegmentOp>
    void forEach(PointOp po, SegmentOp so, bool noCycle = false) const
    {
        ForEach<const Edge*, PointOp, SegmentOp>(this, po, so, noCycle);
    }

    /*
     * apply an operation to every Segment associated with this Edge
     * @param o operation (a function or functor that takes a pointer to
     * a segment as an argument)
     */
    template<typename T>
    void forEachSegment(T o)
    {
        forEach(NoOp<EdgePoint*>(), o);
    }

    /*
     * a version of forEachSegment for const edges
     * @param o an operation on a const Segment
     */
    template<typename T>
    void forEachSegment(T o) const
    {
        forEach(NoOp<const EdgePoint*>(), o);
    }

    /*
     * apply an operation to every EdgePoint associated with this edge
     * @param o operation (a function or functor that takes a pointer to
     * an EdgePoint as an argument)
     * @param noCycle if the edge is a cycle don't apply o to the
     * start/end point twice.
     */
    template<typename T>
    void forEachEdgePoint(T o, bool noCycle = false)
    {
        forEach(o, NoOp<Segment*>(), noCycle);
    }

    /*
     * a version of forEachEdgePoint for const edges
     * @param o an operation on a const EdgePoint
     * @param noCycle if the edge is a cycle apply o to the
     * start/end point only once.
     */
    template<typename T>
    void forEachEdgePoint(T o, bool noCycle = false) const
    {
        forEach(o, NoOp<const Segment*>(), noCycle);
    }

// To prevent C++ objects from being destroyed in garbage collected languages
// when the libraries are called from SWIG, we hide the declarations of the
// destructors and prevent generation of default destructors.
#ifndef SWIG
    /*
     * cleanup segments
     */
    ~Edge()
    {
        forEach(delete_object(), delete_object(), true);
    }
#endif

    /*
     * the sum of the lengths of all the segments
     */
    double pathLength() const;

    /*
     * get a list of all the EdgePoints along the Edge path
     */
    void getPath(ConstEdgePoints& vs) const;

    /*
     * @return a list of the coordinates along the edge route
     */
    straightener::Route* getRoute() const;

    void getTopologyConstraints(std::vector<TopologyConstraint*>* ts) const
    {
        forEach(
            std::bind(&EdgePoint::getBendConstraint, std::placeholders::_1, ts),
            std::bind(
                &Segment::getStraightConstraints,
                std::placeholders::_1,
                ts
            ),
            true
        );
    }

    bool assertConvexBends() const;

    bool cycle() const
    {
        return firstSegment->start == lastSegment->end;
    }

    std::string toString() const;
};

/**
 * @brief  A vector of pointers to Edge objects.
 */
using Edges = std::vector<Edge*>;

}  // namespace topology
