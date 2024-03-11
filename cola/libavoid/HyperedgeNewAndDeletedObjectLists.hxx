///
/// @file HyperedgeNewAndDeletedObjectLists.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace avoid {

//! @brief   The HyperedgeNewAndDeletedObjectLists class stores lists of
//!          objects created and deleted during hyperedge improvement.
//!
//! After hyperedge improvement, this information can be produced by calling
//! the Router::newAndDeletedObjectListsFromHyperedgeImprovement() method.
//!
//! After hyperedge rerouting, this information can be produced by calling
//! the HyperedgeRerouter::newAndDeletedObjectLists() method for each
//! hyperedge being fully rerouted.
//!
//! The HyperedgeNewAndDeletedObjectLists::changedConnectorList attribute
//! will only be used for hyperedge improvement and will always be empty
//! for hyperedge rerouting.
//!
struct HyperedgeNewAndDeletedObjectLists
{
    //! A list of newly created junctions.
    JunctionRefList newJunctionList;

    //! A list of newly created connectors.
    ConnRefList newConnectorList;

    //! A list of deleted junctions.
    JunctionRefList deletedJunctionList;

    //! A list of deleted connectors.
    ConnRefList deletedConnectorList;

    //! A list of changed connectors.
    ConnRefList changedConnectorList;
};

}  // namespace avoid
