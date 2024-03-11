///
/// @file EdgeInf.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __EDGE_INF_HXX_F8B2026FA943__
#define __EDGE_INF_HXX_F8B2026FA943__

#include <list>

namespace avoid {

class Router;

class EdgeInf
{
public:
    EdgeInf(VertInf* v1, VertInf* v2, const bool orthogonal = false);
    ~EdgeInf();

    inline double getDist(void)
    {
        return m_dist;
    }

    void setDist(double dist);
    void alertConns(void);
    void addConn(bool* flag);
    void addCycleBlocker(void);
    void addBlocker(int b);
    bool added(void);
    bool isOrthogonal(void) const;
    bool isDummyConnection(void) const;
    bool isDisabled(void) const;
    void setDisabled(const bool disabled);
    bool rotationLessThan(const VertInf* last, const EdgeInf* rhs) const;
    std::pair<VertID, VertID> ids(void) const;
    std::pair<Point, Point>   points(void) const;
    void                      db_print(void);
    void                      checkVis(void);
    VertInf*                  otherVert(const VertInf* vert) const;
    static EdgeInf*           checkEdgeVisibility(
                  VertInf* i,
                  VertInf* j,
                  bool     knownNew = false
              );
    static EdgeInf* existingEdge(VertInf* i, VertInf* j);
    int             blocker(void) const;

    bool   isHyperedgeSegment(void) const;
    void   setHyperedgeSegment(const bool hyperedge);
    double mtstDist(void) const;
    void   setMtstDist(const double joinCost);

    EdgeInf* lstPrev;
    EdgeInf* lstNext;

private:
    friend class MinimumTerminalSpanningTree;
    friend class VertInf;

    void makeActive(void);
    void makeInactive(void);
    int  firstBlocker(void);
    bool isBetween(VertInf* i, VertInf* j);

    Router*               m_router;
    int                   m_blocker;
    bool                  m_added;
    bool                  m_visible;
    bool                  m_orthogonal;
    bool                  m_isHyperedgeSegment;
    bool                  m_disabled;
    VertInf*              m_vert1;
    VertInf*              m_vert2;
    EdgeInfList::iterator m_pos1;
    EdgeInfList::iterator m_pos2;
    FlagList              m_conns;
    double                m_dist;
    double                m_mtst_dist;
};

using EdgeInfList = std::list<EdgeInf*>;

}  // namespace avoid

#endif  // __EDGE_INF_HXX_F8B2026FA943__
