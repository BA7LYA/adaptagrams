///
/// @file VertInf.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __VERT_INF_HXX_708F5CEA3217__
#define __VERT_INF_HXX_708F5CEA3217__

#include <list>
#include <tuple>

#include "libavoid/ConnDirFlag.hxx"
#include "libavoid/EdgeInf.hxx"
#include "libavoid/Point.hxx"
#include "libavoid/VertID.hxx"

namespace avoid {

class ANode;
class Router;

class VertInf
{
public:
    VertInf(
        Router*       router,
        const VertID& vid,
        const Point&  vpoint,
        const bool    addToRouter = true
    );
    ~VertInf();

    void Reset(const VertID& vid, const Point& vpoint);
    void Reset(const Point& vpoint);
    void removeFromGraph(const bool isConnVert = true);
    bool orphaned(void);

    unsigned int pathLeadsBackTo(const VertInf* start) const;
    void         setVisibleDirections(const ConnDirFlags directions);
    ConnDirFlags directionFrom(const VertInf* other) const;
    // Checks if this vertex has the target as a visibility neighbour.
    EdgeInf*     hasNeighbour(VertInf* target, bool orthogonal) const;
    void         orphan(void);

    VertInf** makeTreeRootPointer(VertInf* root);
    VertInf*  treeRoot(void) const;
    VertInf** treeRootPointer(void) const;
    void      setTreeRootPointer(VertInf** pointer);
    void      clearTreeRootPointer(void);

    void     setSPTFRoot(VertInf* root);
    VertInf* sptfRoot(void) const;

    Router*      _router;
    VertID       id;
    Point        point;
    VertInf*     lstPrev;
    VertInf*     lstNext;
    VertInf*     shPrev;
    VertInf*     shNext;
    EdgeInfList  visList;
    unsigned int visListSize;
    EdgeInfList  orthogVisList;
    unsigned int orthogVisListSize;
    EdgeInfList  invisList;
    unsigned int invisListSize;
    VertInf*     pathNext;

    // The tree root and distance value used when computing MTSTs.
    // XXX: Maybe these should be allocated as a separate struct
    //      and referenced via a pointer.  This would be slower due
    //      to memory allocation, but would save 2 x 8 = 24 bytes per
    //      VertInf on 64-bit machines.
    VertInf*  m_orthogonalPartner;
    VertInf** m_treeRoot;
    double    sptfDist;

    ConnDirFlags      visDirections;
    std::list<ANode*> aStarDoneNodes;
    std::list<ANode*> aStarPendingNodes;
    // Flags for orthogonal visibility properties, i.e., whether the
    // line points to a shape edge, connection point or an obstacle.
    unsigned int      orthogVisPropFlags;
};

using VertexPair = std::pair<VertInf*, VertInf*>;

}  // namespace avoid

#endif  // __VERT_INF_HXX_708F5CEA3217__
