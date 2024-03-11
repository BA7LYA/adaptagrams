///
/// @file TransactionPhases.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace avoid {

//! @brief  Types of routing phases reported by
//!         Router::shouldContinueTransactionWithProgress().
//!
//! This phases will occur in the order given here, but each phase may take
//! varying amounts of time.
//!
enum TransactionPhases
{
    //! @brief  The orthogonal visibility graph is built by conducting a
    //!         scan in each dimension.  This is the x-dimension.
    TransactionPhaseOrthogonalVisibilityGraphScanX = 1,

    //! @brief  The orthogonal visibility graph is built by conducting a
    //!         scan in each dimension.  This is the y-dimension.
    TransactionPhaseOrthogonalVisibilityGraphScanY,

    //! @brief  Initial routes are searched for in the visibility graph.
    TransactionPhaseRouteSearch,

    //! @brief  With crossing penalties enabled, crossing detection is
    //!         performed to find all crossings.
    TransactionPhaseCrossingDetection,

    //! @brief  Crossing connectors are rerouted to search for better routes.
    TransactionPhaseRerouteSearch,

    //! @brief  Orthogonal edge segments are nudged apart in the x-dimension.
    TransactionPhaseOrthogonalNudgingX,

    //! @brief  Orthogonal edge segments are nudged apart in the y-dimension.
    TransactionPhaseOrthogonalNudgingY,

    //! @brief  Not a real phase, but represents the router is finished (or has
    //!         aborted) the transaction and you may interact with is again.
    TransactionPhaseCompleted
};

}  // namespace avoid
