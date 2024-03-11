///
/// @file ConnectorCrossings.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace avoid {

class ConnectorCrossings
{
public:
    ConnectorCrossings(
        Avoid::Polygon& poly,
        bool            polyIsConn,
        Avoid::Polygon& conn,
        ConnRef*        polyConnRef = nullptr,
        ConnRef*        connConnRef = nullptr
    );
    void clear(void);
    void countForSegment(size_t cIndex, const bool finalSegment);

    Avoid::Polygon& poly;
    bool            polyIsConn;
    Avoid::Polygon& conn;
    bool            checkForBranchingSegments;
    ConnRef*        polyConnRef;
    ConnRef*        connConnRef;

    unsigned int    crossingCount;
    unsigned int    crossingFlags;
    PointSet*       crossingPoints;
    PtOrderMap*     pointOrders;
    SharedPathList* sharedPaths;

    double firstSharedPathAtEndLength;
    double secondSharedPathAtEndLength;
};

}  // namespace avoid
