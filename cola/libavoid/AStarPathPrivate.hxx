///
/// @file AStarPathPrivate.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __A_STAR_PATH_PRIVATE_HXX_EE51B752204E__
#define __A_STAR_PATH_PRIVATE_HXX_EE51B752204E__

#include <vector>

#include "libavoid/ANode.hxx"
#include "libavoid/VertInf.hxx"

namespace avoid {

class ConnRef;
class Point;

class AStarPathPrivate
{
public:
    AStarPathPrivate()
        : m_available_nodes()
        , m_available_array_size(0)
        , m_available_array_index(0)
        , m_available_node_index(0)
    {
    }

    ~AStarPathPrivate()
    {
        // Free memory
        for (size_t i = 0; i < m_available_nodes.size(); ++i)
        {
            delete[] m_available_nodes[i];
        }
    }

    // Returns a pointer to an ANode for aStar search, but allocates
    // these in blocks
    ANode* newANode(const ANode& node, const bool addToPending = true)
    {
        const size_t blockSize = 5000;
        if ((m_available_array_index + 1 > m_available_array_size)
            || (m_available_node_index >= blockSize))
        {
            m_available_nodes.push_back(new ANode[blockSize]);
            ++m_available_array_size;
            m_available_node_index  = 0;
            m_available_array_index = m_available_array_size - 1;
        }

        ANode* nodes   = m_available_nodes[m_available_array_index];
        ANode* newNode = &(nodes[m_available_node_index++]);
        *newNode       = node;
        if (addToPending)
        {
            node.inf->aStarPendingNodes.push_back(newNode);
        }
        return newNode;
    }

    // Returns the best path from src to tar using the cost function.
    //
    // The path is worked out using the aStar algorithm, and is encoded via
    // prevNode values for each ANode which point back to the previous ANode.
    // At completion, this order is written into the pathNext links in each
    // of the VerInfs along the path.
    //
    // The aStar STL code is originally based on public domain code available
    // on the internet.
    //
    void search(ConnRef* lineRef, VertInf* src, VertInf* tar, VertInf* start);

private:
    void determineEndPointLocation(
        double   dist,
        VertInf* start,
        VertInf* target,
        VertInf* other,
        int      level
    );
    double estimatedCost(ConnRef* lineRef, const Point* last, const Point& curr)
        const;

    std::vector<ANode*> m_available_nodes;
    size_t              m_available_array_size;
    size_t              m_available_array_index;
    size_t              m_available_node_index;

    // For determining estimated cost target.
    std::vector<VertInf*>     m_cost_targets;
    std::vector<unsigned int> m_cost_targets_directions;
    std::vector<double>       m_cost_targets_displacements;
};

}  // namespace avoid

#endif  // __A_STAR_PATH_PRIVATE_HXX_EE51B752204E__
