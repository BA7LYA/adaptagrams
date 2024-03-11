///
/// @file EdgePoint.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-11
/// @copyright Copyright (c) 2024
///

#include <vector>

namespace topoloty {

class Node;
class BendConstraint;

/*
 * An EdgePoint is a point along an edge path.  It must correspond to
 * either the middle of a Node (the start/end of the edge) or to a corner
 * of a Node (a bend point around an edge).
 */
class EdgePoint
{
public:
    // the node / variable / rectangle associated with this EdgePoint
    Node* node;

    // where the EdgePoint lies on the rectangle
    enum RectIntersect
    {
        TR,     //< top right corner
        BR,     //< bottom right corner
        BL,     //< bottom left corner
        TL,     //< bends around rectangle's top-left corner
        CENTRE  //< connected to the rectangle's centre, hence the end of the
                // edge.
    } rectIntersect;

    // the incoming segment to this EdgePoint on the edge path
    Segment* inSegment;

    // the outgoing segment to this EdgePoint on the edge path
    Segment* outSegment;

    /* each articulation EdgePoint (where isReal()==false)
     *  will be assigned (not immediately) a bendConstraint
     */
    BendConstraint* bendConstraint;

    // append bendConstraint (if not nullptr) to ts
    void getBendConstraint(std::vector<TopologyConstraint*>* ts);

    // @return true if constraint created
    bool createBendConstraint(vpsc::Dim scanDim);

    // delete the bendConstraint and reset pointer to nullptr
    void deleteBendConstraint();

    /*
     * Constructor associates the point with a node vertex but
     * not an edge.
     */
    EdgePoint(Node* n, RectIntersect i)
        : node(n)
        , rectIntersect(i)
        , inSegment(nullptr)
        , outSegment(nullptr)
        , bendConstraint(nullptr)
    {
    }

    /*
     * @param dim the axis (either horizontal or
     * vertical) of the coordinate to return
     * @return the position, computed based on rectIntersect and rectangle
     * vertices of the node
     */
    double pos(vpsc::Dim dim) const;

    // @return x position
    double posX() const
    {
        return pos(vpsc::HORIZONTAL);
    }

    // @return y position
    double posY() const
    {
        return pos(vpsc::VERTICAL);
    }

    /*
     *  @return where the EdgePoint on the rectangle as a vertex index
     *  for libavoid.
     */
    unsigned short rectIntersectAsVertexNumber(void) const
    {
        switch (rectIntersect)
        {
        case topology::EdgePoint::BR: return 0;
        case topology::EdgePoint::TR: return 1;
        case topology::EdgePoint::TL: return 2;
        case topology::EdgePoint::BL: return 3;
        default: return 4;
        }
    }

    /*
     * for any two EdgePoint the following should always be false!
     * @param e an EdgePoint (not this one)
     */
    bool uniqueCheck(const EdgePoint* e) const
    {
        COLA_ASSERT(this != e);
        return node == e->node && rectIntersect == e->rectIntersect;
    }

// To prevent C++ objects from being destroyed in garbage collected languages
// when the libraries are called from SWIG, we hide the declarations of the
// destructors and prevent generation of default destructors.
#ifndef SWIG
    ~EdgePoint();
#endif

    /*
     * @return true if the EdgePoint is the end of an edge otherwise
     * asserts that it is a valid bend point.
     */
    bool isEnd() const;

    /*
     * asserts that, if this is a bend point, it does not double back in either
     * the horizontal or vertical directions.
     */
    bool assertConvexBend() const;

    /*
     * @return offset from centre of node
     */
    double offset(vpsc::Dim scanDim) const;

    /*
     * remove this point from the edge replacing its in and out
     * segments with a single new Segment.
     * @return the replacement Segment
     */
    Segment* prune(vpsc::Dim scanDim);
};

/*
 * A vector of pointers to EdgePoint objects.
 */
using EdgePoints      = std::vector<EdgePoint*>;
using ConstEdgePoints = std::vector<const EdgePoint*>;

}  // namespace topoloty
