///
/// @file PtOrder.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include <map>

namespace avoid {

class ConnRef;
class PtConnPtrPair;
class PointRepVector;
class NodeIndexPairLinkList;
class Point;

class PtOrder
{
public:
    PtOrder();
    ~PtOrder();
    void addPoints(
        const size_t         dim,
        const PtConnPtrPair& arg1,
        const PtConnPtrPair& arg2
    );
    void addOrderedPoints(
        const size_t         dim,
        const PtConnPtrPair& innerArg,
        const PtConnPtrPair& outerArg,
        bool                 swapped
    );
    int            positionFor(const size_t dim, const ConnRef* conn);
    PointRepVector sortedPoints(const size_t dim);

private:
    size_t insertPoint(const size_t dim, const PtConnPtrPair& point);
    void   sort(const size_t dim);

    // One for each dimension.
    bool                  sorted[2];
    PointRepVector        nodes[2];
    NodeIndexPairLinkList links[2];
    PointRepVector        sortedConnVector[2];
};

using PtOrderMap = std::map<avoid::Point, PtOrder>;

}  // namespace avoid
