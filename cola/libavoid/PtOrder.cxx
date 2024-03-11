///
/// @file PtOrder.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/PtOrder.hxx"

namespace avoid {

PtOrder::PtOrder()
{
    // We have sorted neither list initially.
    for (size_t dim = 0; dim < 2; ++dim)
    {
        sorted[dim] = false;
    }
}

PtOrder::~PtOrder() {}

PointRepVector PtOrder::sortedPoints(const size_t dim)
{
    // Sort if not already sorted.
    if (sorted[dim] == false)
    {
        sort(dim);
    }
    return sortedConnVector[dim];
}

int PtOrder::positionFor(const size_t dim, const ConnRef* conn)
{
    // Sort if not already sorted.
    if (sorted[dim] == false)
    {
        sort(dim);
    }

    // Just return position from the sorted list.
    size_t i = 0;
    for (; i < sortedConnVector[dim].size(); ++i)
    {
        if (sortedConnVector[dim][i].second == conn)
        {
            return (int)i;
        }
    }
    return -1;
}

size_t PtOrder::insertPoint(const size_t dim, const PtConnPtrPair& pointPair)
{
    // Is this connector bendpoint already inserted?
    size_t i = 0;
    for (; i < nodes[dim].size(); ++i)
    {
        if (nodes[dim][i].second == pointPair.second)
        {
            return i;
        }
    }
    // Not found, insert.
    nodes[dim].push_back(pointPair);
    return nodes[dim].size() - 1;
}

void PtOrder::addPoints(
    const size_t         dim,
    const PtConnPtrPair& arg1,
    const PtConnPtrPair& arg2
)
{
    // Add points, but not ordering information.
    insertPoint(dim, arg1);
    insertPoint(dim, arg2);
}

void PtOrder::addOrderedPoints(
    const size_t         dim,
    const PtConnPtrPair& innerArg,
    const PtConnPtrPair& outerArg,
    bool                 swapped
)
{
    PtConnPtrPair inner = (swapped) ? outerArg : innerArg;
    PtConnPtrPair outer = (swapped) ? innerArg : outerArg;
    COLA_ASSERT(inner != outer);

    // Add points.
    size_t innerIndex = insertPoint(dim, inner);
    size_t outerIndex = insertPoint(dim, outer);

    // And edge for ordering information.
    links[dim].push_back(std::make_pair(outerIndex, innerIndex));
}

void PtOrder::sort(const size_t dim)
{
    // This is just a topological sort of the points using the edges info.

    sorted[dim] = true;

    size_t n = nodes[dim].size();

    // Build an adjacency matrix for easy lookup.
    std::vector<std::vector<bool>> adjacencyMatrix(n);
    for (size_t i = 0; i < n; ++i)
    {
        adjacencyMatrix[i].assign(n, false);
    }
    std::vector<int>   incomingDegree(n);
    std::queue<size_t> queue;

    // Populate the dependency matrix.
    for (NodeIndexPairLinkList::iterator it = links[dim].begin();
         it != links[dim].end();
         ++it)
    {
        adjacencyMatrix[it->first][it->second] = true;
    }

    // Build incoming degree lookup structure, and add nodes with no
    // incoming edges to queue.
    for (size_t i = 0; i < n; ++i)
    {
        int degree = 0;

        for (size_t j = 0; j < n; ++j)
        {
            if (adjacencyMatrix[j][i])
            {
                degree++;
            }
        }
        incomingDegree[i] = degree;

        if (degree == 0)
        {
            queue.push(i);
        }
    }

    while (queue.empty() == false)
    {
        size_t k = queue.front();
        assert(k < nodes[dim].size());
        queue.pop();

        // Insert node k into the sorted list
        sortedConnVector[dim].push_back(nodes[dim][k]);

        // Remove all edges leaving node k:
        for (size_t i = 0; i < n; ++i)
        {
            if (adjacencyMatrix[k][i])
            {
                adjacencyMatrix[k][i] = false;
                incomingDegree[i]--;

                if (incomingDegree[i] == 0)
                {
                    queue.push(i);
                }
            }
        }
    }
}

}  // namespace avoid
