///
/// @file RoutingOption.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace avoid {

//! @brief  Types of routing options that can be enabled.
enum RoutingOption
{
    //! This option causes the final segments of connectors, which are
    //! attached to shapes, to be nudged apart.  Usually these segments
    //! are fixed, since they are considered to be attached to ports.
    //!
    //! Defaults to false.
    //!
    //! This option also causes routes running through the same checkpoint
    //! to be nudged apart.
    //!
    //! This option has no effect if ::nudgeSharedPathsWithCommonEndPoint is
    //! set to false,
    //!
    //! @note   This will allow routes to be nudged up to the bounds of shapes.
    //!
    nudgeOrthogonalSegmentsConnectedToShapes = 0,

    //! This option causes hyperedge routes to be locally improved fixing
    //! obviously bad paths.  As part of this process libavoid will
    //! effectively move junctions, setting new ideal positions which can be
    //! accessed via JunctionRef::recommendedPosition() for each junction.
    //!
    //! Defaults to true.
    //!
    //! This will not add or remove junctions, so will keep the hyperedge
    //! topology the same.  Better routes can be achieved by enabling the
    //! ::improveHyperedgeRoutesMovingAddingAndDeletingJunctions option.
    //!
    //! If initial sensible positions for junctions in hyperedges are not
    //! known you can register those hyperedges with the HyperedgeRerouter
    //! class for complete rerouting.
    //!
    //! @sa   improveHyperedgeRoutesMovingAddingAndDeletingJunctions
    //! @sa   Router::hyperedgeRerouter()
    //!
    improveHyperedgeRoutesMovingJunctions,

    //! This option penalises and attempts to reroute orthogonal shared
    //! connector paths terminating at a common junction or shape
    //! connection pin.  When multiple connector paths enter or leave
    //! the same side of a junction (or shape pin), the router will
    //! attempt to reroute these to different sides of the junction or
    //! different shape pins.
    //!
    //! Defaults to false.
    //!
    //! This option depends on the ::fixedSharedPathPenalty penalty having
    //! been set.
    //!
    //! @sa     fixedSharedPathPenalty
    //! @note   This option is still experimental!  It is not recommended
    //!         for normal use.
    //!
    penaliseOrthogonalSharedPathsAtConnEnds,

    //! This option can be used to control whether collinear line
    //! segments that touch just at their ends will be nudged apart.
    //! The overlap will usually be resolved in the other dimension,
    //! so this is not usually required.
    //!
    //! Defaults to false.
    //!
    nudgeOrthogonalTouchingColinearSegments,

    //! This option can be used to control whether the router performs
    //! a preprocessing step before orthogonal nudging where is tries
    //! to unify segments and centre them in free space.  This
    //! generally results in better quality ordering and nudging.
    //!
    //! Defaults to true.
    //!
    //! You may wish to turn this off for large examples where it
    //! can be very slow and will make little difference.
    //!
    performUnifyingNudgingPreprocessingStep,

    //! This option causes hyperedge routes to be locally improved fixing
    //! obviously bad paths.
    //!
    //! It can cause junctions and connectors to be added or removed from
    //! hyperedges.  To get details of these changes for each connector you can
    //! call Router::newAndDeletedObjectListsFromHyperedgeImprovement().
    //!
    //! As part of this process libavoid will effectively move junctions by
    //! setting new ideal positions for each remaining or added junction,
    //! which can be read from JunctionRef::recommendedPosition() for each
    //! junction.
    //!
    //! Defaults to false.
    //!
    //! If set, this option overrides the
    //! ::improveHyperedgeRoutesMovingJunctions option.
    //!
    //! If initial sensible positions for junctions in hyperedges are not
    //! known you can register those hyperedges with the HyperedgeRerouter
    //! class for complete rerouting.
    //!
    //! @sa   improveHyperedgeRoutesMovingJunctions
    //! @sa   Router::hyperedgeRerouter()
    //!
    improveHyperedgeRoutesMovingAddingAndDeletingJunctions,

    //! This option determines whether intermediate segments of connectors that
    //! are attached to common endpoints will be nudged apart.  Usually these
    //! segments get nudged apart, but you may want to turn this off if you
    //! would prefer that entire shared paths terminating at a common end point
    //! should overlap.
    //!
    //! Defaults to true.
    //!
    nudgeSharedPathsWithCommonEndPoint,

    // Used for determining the size of the routing options array.
    // This should always we the last value in the enum.
    lastRoutingOptionMarker
};

}  // namespace avoid
