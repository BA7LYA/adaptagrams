///
/// @file RoutingParameter.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace avoid {

//! @brief  Types of routing parameters and penalties that can be used to
//!         tailor the style and improve the quality of the connector
//!         routes produced.
enum RoutingParameter
{
    //! @brief  This penalty is applied for each segment in the connector
    //!         path beyond the first.  This should always normally be set
    //!         when doing orthogonal routing to prevent step-like connector
    //!         paths.
    //! @note   This penalty must be set (i.e., be greater than zero) in
    //!         order for orthogonal connector nudging to be performed, since
    //!         this requires reasonable initial routes.
    segmentPenalty = 0,

    //! @brief  This penalty is applied in its full amount to tight acute
    //!         bends in the connector path.  A smaller portion of the penalty
    //!         is applied for slight bends, i.e., where the bend is close to
    //!         180 degrees.  This is useful for polyline routing where there
    //!         is some evidence that tighter corners are worse for
    //!         readability, but that slight bends might not be so bad,
    //!         especially when smoothed by curves.
    anglePenalty,

    //! @brief  This penalty is applied whenever a connector path crosses
    //!         another connector path.  It takes shared paths into
    //!         consideration and the penalty is only applied if there
    //!         is an actual crossing.
    //! @note   This penalty is still experimental!  It is not recommended
    //!         for normal use.
    crossingPenalty,

    //! @brief  This penalty is applied whenever a connector path crosses
    //!         a cluster boundary.
    //! @note   This penalty is still experimental!  It is not recommended
    //!         for normal use.
    //! @note   This penalty is very slow.  You can override the method
    //!         Router::shouldContinueTransactionWithProgress() to check
    //!         progress and possibly cancel overly slow transactions.
    clusterCrossingPenalty,

    //! @brief  This penalty is applied whenever a connector path shares
    //!         some segments with an immovable portion of an existing
    //!         connector route (such as the first or last segment of a
    //!         connector).
    //! @note   This penalty is still experimental!  It is not recommended
    //!         for normal use.
    fixedSharedPathPenalty,

    //! @brief  This penalty is applied to port selection choice when the
    //!         other end of the connector being routed does not appear in
    //!         any of the 90 degree visibility cones centered on the
    //!         visibility directions for the port.
    //! @note   This penalty is still experimental!  It is not recommended
    //!         for normal use.
    //! @note   This penalty is very slow.  You can override the method
    //!         Router::shouldContinueTransactionWithProgress() to check
    //!         progress and possibly cancel overly slow transactions.
    portDirectionPenalty,

    //! @brief This parameter defines the spacing distance that will be added
    //!        to the sides of each shape when determining obstacle sizes for
    //!        routing.  This controls how closely connectors pass shapes, and
    //!        can be used to prevent connectors overlapping with shape
    //!        boundaries. By default, this distance is set to a value of 0.
    shapeBufferDistance,

    //! @brief This parameter defines the spacing distance that will be used
    //!        for nudging apart overlapping corners and line segments of
    //!        connectors.  By default, this distance is set to a value of 4.
    idealNudgingDistance,

    //! @brief  This penalty is applied whenever a connector path travels
    //!         in the direction opposite of the destination from the source
    //!         endpoint.  By default this penalty is set to zero.  This
    //!         shouldn't be needed in most cases but can be useful if you
    //!         use penalties such as ::crossingPenalty which cause connectors
    //!         to loop around obstacles.
    reverseDirectionPenalty,

    // Used for determining the size of the routing parameter array.
    // This should always we the last value in the enum.
    lastRoutingParameterMarker
};

// Backwards compatibility
using PenaltyType = enum RoutingParameter;

}  // namespace avoid
