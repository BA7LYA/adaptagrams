///
/// @file Checkpoint.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/dllexport.hxx"

namespace avoid {

//! @brief  A checkpoint is a point that the route for a particular connector
//!         must visit.  They may optionally be given an arrival/departure
//!         direction.
//!
class AVOID_EXPORT Checkpoint
{
public:
    //! @brief  A point that a route must visit.
    //!
    //! The connector will be able to enter and leave this checkpoint from
    //! any direction.
    //!
    //! @param[in] p  The Point that must be visited.
    Checkpoint(const Point& p)
        : point(p)
        , arrivalDirections(ConnDirAll)
        , departureDirections(ConnDirAll)
    {
    }

    //! @brief  A point that a route must visit.
    //!
    //! The connector will be able to enter and leave this checkpoint from
    //! the directions specified.  Give Avoid::ConnDirAll to specify all
    //! directions.
    //!
    //! @param[in] p  The Point that must be visited.
    //! @param[in] ad Avoid::ConnDirFlags denoting arrival directions for
    //!               the connector portion leading up to this checkpoint.
    //! @param[in] dd Avoid::ConnDirFlags denoting departure directions
    //!               for the connector portion leading away from this
    //!               checkpoint.
    Checkpoint(const Point& p, ConnDirFlags ad, ConnDirFlags dd)
        : point(p)
        , arrivalDirections(ad)
        , departureDirections(dd)
    {
    }

    // Default constructor.
    Checkpoint()
        : point(Point())
        , arrivalDirections(ConnDirAll)
        , departureDirections(ConnDirAll)
    {
    }

    Point        point;
    ConnDirFlags arrivalDirections;
    ConnDirFlags departureDirections;
};

}  // namespace avoid
