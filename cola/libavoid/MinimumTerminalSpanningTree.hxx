///
/// @file MinimumTerminalSpanningTree.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __MINIMUM_TERMINAL_SPANNING_TREE_HXX_49FA5D65672D__
#define __MINIMUM_TERMINAL_SPANNING_TREE_HXX_49FA5D65672D__

namespace avoid {

// This class is not intended for public use.
// It is used by the hyperedge routing code to build a minimum terminal
// spanning tree for a set of terminal vertices.
class MinimumTerminalSpanningTree
{
public:
    MinimumTerminalSpanningTree(
        Router*                       router,
        std::set<VertInf*>            terminals,
        JunctionHyperedgeTreeNodeMap* hyperedgeTreeJunctions = nullptr
    );
    ~MinimumTerminalSpanningTree();

    // Uses Interleaved construction of the MTST and SPTF (heuristic 2
    // from paper).  This is the preferred construction approach.
    void constructInterleaved(void);
    // Uses Sequential construction of the MTST (heuristic 1 from paper).
    void constructSequential(void);

    void               setDebuggingOutput(FILE* fp, unsigned int counter);
    HyperedgeTreeNode* rootJunction(void) const;

private:
    void buildHyperedgeTreeToRoot(
        VertInf*           curr,
        HyperedgeTreeNode* prevNode,
        VertInf*           prevVert,
        bool               markEdges = false
    );
    VertInf** resetDistsForPath(VertInf* currVert, VertInf** newRootVertPtr);
    void      rewriteRestOfHyperedge(VertInf* vert, VertInf** newTreeRootPtr);
    void      drawForest(VertInf* vert, VertInf* prev);

    void                    makeSet(VertInf* vertex);
    VertexSetList::iterator findSet(VertInf* vertex);
    void unionSets(VertexSetList::iterator s1, VertexSetList::iterator s2);
    HyperedgeTreeNode* addNode(VertInf* vertex, HyperedgeTreeNode* prevNode);

    void removeInvalidBridgingEdges(void);
    void commitToBridgingEdge(EdgeInf* e);
    bool connectsWithoutBend(VertInf* oldLeaf, VertInf* newLeaf);
    LayeredOrthogonalEdgeList getOrthogonalEdgesFromVertex(
        VertInf* vert,
        VertInf* prev
    );
    VertInf*   orthogonalPartner(VertInf* vert, double penalty = 0);
    VertexPair realVerticesCountingPartners(EdgeInf* edge);

    Router*                       router;
    bool                          isOrthogonal;
    std::set<VertInf*>            terminals;
    std::set<VertInf*>            origTerminals;
    JunctionHyperedgeTreeNodeMap* hyperedgeTreeJunctions;

    VertexNodeMap        nodes;
    HyperedgeTreeNode*   m_rootJunction;
    double               bendPenalty;
    VertexSetList        allsets;
    std::list<VertInf*>  visitedVertices;
    std::list<VertInf*>  extraVertices;
    std::list<VertInf*>  unusedVertices;
    std::list<VertInf**> rootVertexPointers;

    // Vertex heap for extended Dijkstra's algorithm.
    std::vector<VertInf*> vHeap;
    HeapCmpVertInf        vHeapCompare;

    // Bridging edge heap for the extended Kruskal's algorithm.
    std::vector<EdgeInf*> beHeap;
    CmpEdgeInf            beHeapCompare;

    const VertID dimensionChangeVertexID;
};

}  // namespace avoid

#endif  // __MINIMUM_TERMINAL_SPANNING_TREE_HXX_49FA5D65672D__
