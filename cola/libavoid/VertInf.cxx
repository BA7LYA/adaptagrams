///
/// @file VertInf.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/VertInf.hxx"

namespace avoid {

VertInf::VertInf(
    Router*       router,
    const VertID& vid,
    const Point&  vpoint,
    const bool    addToRouter
)
    : _router(router)
    , id(vid)
    , point(vpoint)
    , lstPrev(nullptr)
    , lstNext(nullptr)
    , shPrev(nullptr)
    , shNext(nullptr)
    , visListSize(0)
    , orthogVisListSize(0)
    , invisListSize(0)
    , pathNext(nullptr)
    , m_orthogonalPartner(nullptr)
    , m_treeRoot(nullptr)
    , visDirections(ConnDirNone)
    , orthogVisPropFlags(0)
{
    point.id = vid.objID;
    point.vn = vid.vn;

    if (addToRouter)
    {
        _router->vertices.addVertex(this);
    }
}

VertInf::~VertInf()
{
    COLA_ASSERT(orphaned());
}

EdgeInf* VertInf::hasNeighbour(VertInf* target, bool orthogonal) const
{
    const EdgeInfList& visEdgeList     = (orthogonal) ? orthogVisList : visList;
    EdgeInfList::const_iterator finish = visEdgeList.end();
    for (EdgeInfList::const_iterator edge = visEdgeList.begin(); edge != finish;
         ++edge)
    {
        if ((*edge)->otherVert(this) == target)
        {
            return *edge;
        }
    }
    return nullptr;
}

void VertInf::Reset(const VertID& vid, const Point& vpoint)
{
    id       = vid;
    point    = vpoint;
    point.id = id.objID;
    point.vn = id.vn;
}

void VertInf::Reset(const Point& vpoint)
{
    point    = vpoint;
    point.id = id.objID;
    point.vn = id.vn;
}

// Returns true if this vertex is not involved in any (in)visibility graphs.
bool VertInf::orphaned(void)
{
    return (visList.empty() && invisList.empty() && orthogVisList.empty());
}

void VertInf::removeFromGraph(const bool isConnVert)
{
    if (isConnVert)
    {
        COLA_ASSERT(id.isConnPt());
    }

    // For each vertex.
    EdgeInfList::const_iterator finish = visList.end();
    EdgeInfList::const_iterator edge;
    while ((edge = visList.begin()) != finish)
    {
        // Remove each visibility edge
        (*edge)->alertConns();
        delete (*edge);
    }

    finish = orthogVisList.end();
    while ((edge = orthogVisList.begin()) != finish)
    {
        // Remove each orthogonal visibility edge.
        (*edge)->alertConns();
        delete (*edge);
    }

    finish = invisList.end();
    while ((edge = invisList.begin()) != finish)
    {
        // Remove each invisibility edge
        delete (*edge);
    }
}

void VertInf::orphan(void)
{
    // For each vertex.
    EdgeInfList::const_iterator finish = visList.end();
    EdgeInfList::const_iterator edge;
    while ((edge = visList.begin()) != finish)
    {
        // Remove each visibility edge
        (*edge)->makeInactive();
    }

    finish = orthogVisList.end();
    while ((edge = orthogVisList.begin()) != finish)
    {
        // Remove each orthogonal visibility edge.
        (*edge)->makeInactive();
    }

    finish = invisList.end();
    while ((edge = invisList.begin()) != finish)
    {
        // Remove each invisibility edge
        (*edge)->makeInactive();
    }
}

// Returns the direction of this vertex relative to the other specified vertex.
//
ConnDirFlags VertInf::directionFrom(const VertInf* other) const
{
    double epsilon    = 0.000001;
    Point  thisPoint  = point;
    Point  otherPoint = other->point;
    Point  diff       = thisPoint - otherPoint;

    ConnDirFlags directions = ConnDirNone;
    if (diff.y > epsilon)
    {
        directions |= ConnDirUp;
    }
    if (diff.y < -epsilon)
    {
        directions |= ConnDirDown;
    }
    if (diff.x > epsilon)
    {
        directions |= ConnDirRight;
    }
    if (diff.x < -epsilon)
    {
        directions |= ConnDirLeft;
    }
    return directions;
}

// Given a set of directions, mark visibility edges in all other directions
// as being invalid so they get ignored during the search.
//
void VertInf::setVisibleDirections(const ConnDirFlags directions)
{
    for (EdgeInfList::const_iterator edge = visList.begin();
         edge != visList.end();
         ++edge)
    {
        if (directions == ConnDirAll)
        {
            (*edge)->setDisabled(false);
        }
        else
        {
            VertInf*     otherVert = (*edge)->otherVert(this);
            ConnDirFlags direction = otherVert->directionFrom(this);
            bool         visible   = (direction & directions);
            (*edge)->setDisabled(!visible);
        }
    }

    for (EdgeInfList::const_iterator edge = orthogVisList.begin();
         edge != orthogVisList.end();
         ++edge)
    {
        if (directions == ConnDirAll)
        {
            (*edge)->setDisabled(false);
        }
        else
        {
            VertInf*     otherVert = (*edge)->otherVert(this);
            ConnDirFlags direction = otherVert->directionFrom(this);
            bool         visible   = (direction & directions);
            (*edge)->setDisabled(!visible);
        }
    }
}

// Number of points in path from end back to start, or zero if no path exists.
//
unsigned int VertInf::pathLeadsBackTo(const VertInf* start) const
{
    unsigned int pathlen = 1;
    for (const VertInf* i = this; i != start; i = i->pathNext)
    {
        if ((pathlen > 1) && (i == this))
        {
            // We have a circular path, so path not found.
            return 0;
        }

        pathlen++;
        if (i == nullptr)
        {
            // Path not found.
            return 0;
        }

        // Check we don't have an apparent infinite connector path.
        COLA_ASSERT(pathlen < 2'0000);
    }
    return pathlen;
}

VertInf** VertInf::makeTreeRootPointer(VertInf* root)
{
    m_treeRoot  = (VertInf**)malloc(sizeof(VertInf*));
    *m_treeRoot = root;
    return m_treeRoot;
}

VertInf* VertInf::treeRoot(void) const
{
    return (m_treeRoot) ? *m_treeRoot : nullptr;
}

VertInf** VertInf::treeRootPointer(void) const
{
    return m_treeRoot;
}

void VertInf::clearTreeRootPointer(void)
{
    m_treeRoot = nullptr;
}

void VertInf::setTreeRootPointer(VertInf** pointer)
{
    m_treeRoot = pointer;
}

void VertInf::setSPTFRoot(VertInf* root)
{
    // Use the m_treeRoot instance var, but as just a normal VertInf pointer.
    m_treeRoot = (VertInf**)root;
}

VertInf* VertInf::sptfRoot(void) const
{
    // Use the m_treeRoot instance var, but as just a normal VertInf pointer.
    return (VertInf*)m_treeRoot;
}

}  // namespace avoid
