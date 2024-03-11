/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libavoid - Fast, Incremental, Object-avoiding Line Router
 *
 * Copyright (C) 2004-2015  Monash University
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * See the file LICENSE.LGPL distributed with the library.
 *
 * Licensees holding a valid commercial license may use this file in
 * accordance with the commercial license agreement provided with the
 * library.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author(s):  Michael Wybrow
 */

//! @file    router.h
//! @brief   Contains the interface for the Router class.

#ifndef AVOID_ROUTER_H
#define AVOID_ROUTER_H

#include <ctime>
#include <list>
#include <string>
#include <utility>

#include "libavoid/ActionInfo.hxx"
#include "libavoid/connector.h"
#include "libavoid/dllexport.h"
#include "libavoid/graph.h"
#include "libavoid/hyperedge.h"
#include "libavoid/hyperedgeimprover.h"
#include "libavoid/timer.h"
#include "libavoid/vertices.h"

namespace avoid {

using IntList = std::list<unsigned int>;

class ShapeRef;
class JunctionRef;
class ClusterRef;
using ClusterRefList = std::list<ClusterRef*>;
class Obstacle;
using ObstacleList = std::list<Obstacle*>;
class DebugHandler;

static const unsigned int runningTo        = 1;
static const unsigned int runningFrom      = 2;
static const unsigned int runningToAndFrom = runningTo | runningFrom;

static const double zeroParamValue           = 0;
static const double chooseSensibleParamValue = -1;

//! @brief   The Router class represents a libavoid router instance.
//!
//! Usually you would keep a separate Router instance for each diagram
//! or layout you have open in your application.
//
class AVOID_EXPORT Router
{
public:
    //! @brief  Constructor for router instance.
    //!
    //! @param[in]  flags  One or more Avoid::RouterFlag options to
    //!                    control the behaviour of the router.
    Router(const unsigned int flags);

    //! @brief  Destructor for router instance.
    //!
    //! @note   Destroying a router instance will delete all remaining
    //!         shapes and connectors, thereby invalidating any existing
    //!         pointers to them.
    virtual ~Router();

    ObstacleList   m_obstacles;
    ConnRefList    connRefs;
    ClusterRefList clusterRefs;
    EdgeList       visGraph;
    EdgeList       invisGraph;
    EdgeList       visOrthogGraph;
    ContainsMap    contains;
    VertInfList    vertices;
    ContainsMap    enclosingClusters;

    bool PartialTime;
    bool SimpleRouting;
    bool ClusteredRouting;

    // Poly-line routing options:
    bool IgnoreRegions;
    bool UseLeesAlgorithm;
    bool InvisibilityGrph;

    // General routing options:
    bool SelectiveReroute;

    bool PartialFeedback;
    bool RubberBandRouting;

    // Instrumentation:
#ifdef AVOID_PROFILE
    Timer timers;
#endif

    int st_checked_edges;

    //! @brief Allows setting of the behaviour of the router in regard
    //!        to transactions.  This controls whether transactions are
    //!        used to queue changes and process them efficiently at once
    //!        or they are instead processed immediately.
    //!
    //! It is more efficient to perform actions like shape movement,
    //! addition or deletion as batch tasks, and reroute the necessary
    //! connectors just once after these actions have been performed.
    //! For this reason, libavoid allows you to group such actions into
    //! "transactions" that are processed efficiently when the
    //! processTransaction() method is called.
    //!
    //! By default, the router will process all actions as transactions.
    //! If transactionUse() is set to false, then all actions will get
    //! processed immediately, and cause immediate routing callbacks to
    //! all affected connectors after each action.
    //!
    //! @param[in]  transactions  A boolean value specifying whether to
    //!                           use transactions.
    //!
    void setTransactionUse(const bool transactions);

    //! @brief Reports whether the router groups actions into transactions.
    //!
    //! @return A boolean value describing whether transactions are in use.
    //!
    //! @sa setTransactionUse
    //! @sa processTransaction
    //!
    bool transactionUse(void) const;

    //! @brief Finishes the current transaction and processes all the
    //!        queued object changes efficiently.
    //!
    //! This method will efficiently process all moves, additions and
    //! deletions that have occurred since processTransaction() was
    //! last called.
    //!
    //! If transactionUse() is false, then all actions will have been
    //! processed immediately and this method will do nothing.
    //!
    //! @return A boolean value describing whether there were any actions
    //!         to process.
    //!
    //! @sa setTransactionUse
    //!
    bool processTransaction(void);

    //! @brief Delete a shape from the router scene.
    //!
    //! Connectors that could have a better (usually shorter) path after
    //! the removal of this shape will be marked as needing to be rerouted.
    //!
    //! If the router is using transactions, then this action will occur
    //! the next time Router::processTransaction() is called.  See
    //! Router::setTransactionUse() for more information.
    //!
    //! You should not use the shape reference again after this call.
    //! The router will handle freeing of the shape's memory.
    //!
    //! @param[in]  shape  Pointer reference to the shape being removed.
    //!
    void deleteShape(ShapeRef* shape);

    //! @brief Move or resize an existing shape within the router scene.
    //!
    //! A new polygon for the shape can be given to effectively move or
    //! resize the shape with the scene.  Connectors that intersect the
    //! new shape polygon, or that could have a better (usually shorter)
    //! path after the change, will be marked as needing to be rerouted.
    //!
    //! If the router is using transactions, then this action will occur
    //! the next time Router::processTransaction() is called.  See
    //! Router::setTransactionUse() for more information.
    //!
    //! @param[in]  shape       Pointer reference to the shape being
    //!                         moved/resized.
    //! @param[in]  newPoly     The new polygon boundary for the shape.
    //! @param[in]  first_move  This option is used for some advanced
    //!                         (currently undocumented) behaviour and
    //!                         it should be ignored for the moment.
    //!
    void moveShape(
        ShapeRef*      shape,
        const Polygon& newPoly,
        const bool     first_move = false
    );

    //! @brief Move an existing shape within the router scene by a relative
    //!        distance.
    //!
    //! Connectors that intersect the shape's new position, or that could
    //! have a better (usually shorter) path after the change, will be
    //! marked as needing to be rerouted.
    //!
    //! If the router is using transactions, then this action will occur
    //! the next time Router::processTransaction() is called.  See
    //! Router::setTransactionUse() for more information.
    //!
    //! @param[in]  shape       Pointer reference to the shape being moved.
    //! @param[in]  xDiff       The distance to move the shape in the
    //!                         x dimension.
    //! @param[in]  yDiff       The distance to move the shape in the
    //!                         y dimension.
    //!
    void moveShape(ShapeRef* shape, const double xDiff, const double yDiff);

    //! @brief Remove a junction from the router scene.
    //!
    //! If the router is using transactions, then this action will occur
    //! the next time Router::processTransaction() is called.  See
    //! Router::setTransactionUse() for more information.
    //!
    //! You should not use the junction reference again after this call.
    //! The router will handle freeing of the junction's memory.
    //!
    //! @param[in]  junction  Pointer reference to the junction being
    //!                       removed.
    //!
    void deleteJunction(JunctionRef* junction);

    //! @brief Remove a connector from the router scene.
    //!
    //! If the router is using transactions, then this action will occur
    //! the next time Router::processTransaction() is called.  See
    //! Router::setTransactionUse() for more information.
    //!
    //! You should not use the connector reference again after this call.
    //! The router will handle freeing of the connector's memory.
    //!
    //! @param[in]  connector  Pointer reference to the connector being
    //!                        removed.
    //!
    void deleteConnector(ConnRef* connector);

    //! @brief Move an existing junction within the router scene.
    //!
    //! Connectors that are attached to this junction will be rerouted
    //! as a result of the move.
    //!
    //! If the router is using transactions, then this action will occur
    //! the next time Router::processTransaction() is called.  See
    //! Router::setTransactionUse() for more information.
    //!
    //! @param[in]  junction     Pointer reference to the junction being
    //!                          moved.
    //! @param[in]  newPosition  The new position for the junction.
    //!
    void moveJunction(JunctionRef* junction, const Point& newPosition);

    //! @brief Move an existing junction within the router scene by a
    //!        relative distance.
    //!
    //! Connectors that are attached to this junction will be rerouted
    //! as a result of the move.
    //!
    //! If the router is using transactions, then this action will occur
    //! the next time Router::processTransaction() is called.  See
    //! Router::setTransactionUse() for more information.
    //!
    //! @param[in]  junction    Pointer reference to the junction being
    //!                         moved.
    //! @param[in]  xDiff       The distance to move the junction in the
    //!                         x dimension.
    //! @param[in]  yDiff       The distance to move the junction in the
    //!                         y dimension.
    //!
    void moveJunction(
        JunctionRef* junction,
        const double xDiff,
        const double yDiff
    );

    //! @brief  Sets values for routing parameters, including routing
    //!         penalties.
    //!
    //! libavoid uses a set of parameters to allow the user more control
    //! over routing style and quality.  These different parameters are
    //! described and explained by the RoutingParameter enum.  All
    //! parameters have sensible defaults.
    //!
    //! Regarding routing penalties, libavoid will by default produce
    //! shortest path routes between the source and destination points
    //! for each connector.  There are several penalties that can be
    //! applied during this stage to penalise certain conditions and
    //! thus improve the aesthetics of the routes generated.
    //!
    //! If a value of zero or Avoid::zeroParamValue is given then the
    //! particular parameter value or penalty will be removed.  If no
    //! parameter value argument (or a negative value) is specified
    //! when calling this method, then a sensible penalty value will
    //! be automatically chosen.
    //!
    //! This method does not re-trigger processing of connectors. The new
    //! parameter value will be used the next time rerouting is performed.
    //!
    //! @param[in] parameter  The type of penalty, a RoutingParameter.
    //! @param[in] value      The value to be set for that parameter.
    //!
    void setRoutingParameter(
        const RoutingParameter parameter,
        const double           value = chooseSensibleParamValue
    );

    //! @brief  Returns the current value for a particular routing
    //!         parameter of a given type.
    //!
    //! @param[in] parameter  The type of parameter, a RoutingParameter.
    //! @return  The value for the specified routing parameter.
    //!
    double routingParameter(const RoutingParameter parameter) const;

    //! @brief  Turn specific routing options on or off.
    //!
    //! @param[in] option  The type of routing option, a RoutingOption.
    //! @param[in] value   A boolean representing the option state.
    //!
    void setRoutingOption(const RoutingOption option, const bool value);

    //! @brief  Returns the current state for a specific routing option.
    //!
    //! @param[in] option  The type of routing option, a RoutingOption.
    //! @return  A boolean representing the option state.
    //!
    bool routingOption(const RoutingOption option) const;

    //! @brief  Sets or removes penalty values that are applied during
    //!         connector routing.
    //!
    //! @note   This is a convenience wrapper for the setRoutingParameter()
    // method.  See its documentation for more details.
    //!
    //! @param[in] penType  The type of penalty, a RoutingParameter.
    //! @param[in] penVal   The value to be applied for each occurrence
    //!                     of the penalty case.
    //!
    void setRoutingPenalty(
        const RoutingParameter penType,
        const double           penVal = chooseSensibleParamValue
    );

    //! @brief  Returns a pointer to the hyperedge rerouter for the router.
    //!
    //! @return  A HyperedgeRerouter object that can be used to register
    //!          hyperedges for rerouting.
    //!
    HyperedgeRerouter* hyperedgeRerouter(void);

    //! @brief  Generates an SVG file containing debug output and code that
    //!         can be used to regenerate the instance.
    //!
    //! If transactions are being used, then this method should be called
    //! after processTransaction() has been called, so that it includes any
    //! changes being queued by the router.
    //!
    //! @param[in] filename  A string indicating the filename (without
    //!                      extension) for the output file.  Defaults to
    //!                      "libavoid-debug.svg" if no filename is given.
    //!
    void outputInstanceToSVG(std::string filename = std::string());

    //! @brief  Returns the object ID used for automatically generated
    //!         objects, such as during hyperedge routing.
    //!
    //! Reimplement this in a subclass to set specific IDs for new objects.
    //!
    //! @note   Your implementation should return a value that does not
    //!         fail objectIdIsUnused().
    //!
    //! @return  The ID for a new object.
    //!
    virtual unsigned int newObjectId(void) const;

    //! @brief  Returns whether or not the given ID is already used.
    //!
    //! You should only need this if you reimplement newObjectId().
    //!
    //! @param[in]  id  An ID to test.
    //! @return  A boolean denoting that the given ID is unused.
    //!
    bool objectIdIsUnused(const unsigned int id) const;

    //! @brief  A method called at regular intervals during transaction
    //!         processing to report progress and ask if the Router
    //!         should continue the transaction.
    //!
    //! You can subclass the Avoid::Router class to implement your
    //! own behaviour, such as to show a progress bar or cancel the
    //! transaction at the user's request.
    //!
    //! Note that you can get a sense of progress by looking at the
    //! phaseNumber divided by the totalPhases and the progress in the
    //! current phase, but be aware that phases and the intervals and
    //! proportions at which this method is called will vary, sometime
    //! unpredictably.
    //!
    //! You can return false to request that the Router abort the current
    //! transaction.  Be aware that it may not abort in some phases. For
    //! others it may need to clean up some state before it is safe for
    //! you to interact with it again.  Hence you should wait for a final
    //! call to this method with the phase Avoid::TransactionPhaseCompleted
    //! before continuing.
    //!
    //! @note  Your implementation of this method should be very fast as
    //!        it will be called many times.  Also, you should not change
    //!        or interact with the Router instance at all during these
    //!        calls.  Wait till you have received a call with the
    //!        Avoid::TransactionPhaseCompleted phase.
    //!
    //! @param  elapsedTime  The number of msec spent on the transaction
    //!                      since it began.
    //! @param  phaseNumber  A Router::TransactionPhases representing the
    //!                      current phase of the transaction.
    //! @param  totalPhases  The total number of phases to be performed
    //!                      during the transaction.
    //! @param  proportion   A double representing the progress in the
    //!                      current phase.  Value will be between 0--1.
    //!
    //! @return  Whether the router should continue the transaction.
    //!          This is true in the default (empty) implementation.
    virtual bool shouldContinueTransactionWithProgress(
        unsigned int elapsedTime,
        unsigned int phaseNumber,
        unsigned int totalPhases,
        double       proportion
    );

    //! @brief  Returns a HyperedgeNewAndDeletedObjectLists detailing the
    //!         lists of junctions and connectors created and deleted
    //!         during hyperedge improvement.
    //!
    //! This method will only return information once the router has
    //! processed the transaction.  You should read and act on this
    //! information before processTransaction() is called again.
    //!
    //! After calling this you should no longer refer to any of the
    //! objects in the "deleted" lists --- the router will delete these
    //! and free their memory at its convenience.
    //!
    //! @return A HyperedgeNewAndDeletedObjectLists containing lists of
    //!         junctions and connectors created and deleted.
    //!
    HyperedgeNewAndDeletedObjectLists
        newAndDeletedObjectListsFromHyperedgeImprovement(void) const;

    void          setDebugHandler(DebugHandler* handler);
    DebugHandler* debugHandler(void) const;

    // Processes the actions list for the transaction.  You shouldn't
    // need to cal this.  Instead use processTransaction().
    void processActions(void);

    void deleteCluster(ClusterRef* cluster);
    void attachedShapes(
        IntList&           shapes,
        const unsigned int shapeId,
        const unsigned int type
    );
    void attachedConns(
        IntList&           conns,
        const unsigned int shapeId,
        const unsigned int type
    );
    void markPolylineConnectorsNeedingReroutingForDeletedObstacle(
        Obstacle* obstacle
    );
    void      generateContains(VertInf* pt);
    void      printInfo(void);
    void      regenerateStaticBuiltGraph(void);
    void      destroyOrthogonalVisGraph(void);
    void      setStaticGraphInvalidated(const bool invalidated);
    ConnType  validConnType(const ConnType select = ConnType_None) const;
    bool      isInCrossingPenaltyReroutingStage(void) const;
    void      markAllObstaclesAsMoved(void);
    ShapeRef* shapeContainingPoint(const Point& point);
    void      performContinuationCheck(
             unsigned int phaseNumber,
             size_t       stepNumber,
             size_t       totalSteps
         );
    void registerSettingsChange(void);

    /**
     *  @brief  Set an addon for doing orthogonal topology improvement.
     *
     *  It is expected that you would use the topology::AvoidTopologyAddon()
     *  from libtopology rather than write your own.  This is done so that
     *  libavoid does not have to depend on libtopology.
     */
    void setTopologyAddon(TopologyAddonInterface* topologyAddon);
    void improveOrthogonalTopology(void);

    // Testing and debugging methods.
    bool existsOrthogonalSegmentOverlap(const bool atEnds = false);
    bool existsOrthogonalFixedSegmentOverlap(const bool atEnds = false);
    bool existsOrthogonalTouchingPaths(void);
    int  existsCrossings(const bool optimisedForConnectorType = false);
    bool existsInvalidOrthogonalPaths(void);

    // Outputs the current diagram.  Used for visualising individual
    // steps of various algorithms.  lineReps can be used to draw
    // attention to specific parts of the diagram that have changed
    // between steps.
    void outputDiagramSVG(
        std::string instanceName = std::string(),
        LineReps*   lineReps     = nullptr
    );

    void outputDiagramText(std::string instanceName = std::string());
    void outputDiagram(std::string instanceName = std::string());

private:
    friend class ShapeRef;
    friend class ConnRef;
    friend class JunctionRef;
    friend class Obstacle;
    friend class ClusterRef;
    friend class ShapeConnectionPin;
    friend class MinimumTerminalSpanningTree;
    friend class ConnEnd;
    friend struct HyperedgeTreeNode;
    friend class HyperedgeRerouter;
    friend class HyperedgeImprover;

    unsigned int assignId(const unsigned int suggestedId);

    void addShape(ShapeRef* shape);
    void addJunction(JunctionRef* junction);
    void addCluster(ClusterRef* cluster);
    void modifyConnector(ConnRef* conn);
    void modifyConnector(
        ConnRef*       conn,
        unsigned int   type,
        const ConnEnd& connEnd,
        bool           connPinUpdate = false
    );
    void modifyConnectionPin(ShapeConnectionPin* pin);

    void removeObjectFromQueuedActions(const void* object);
    void newBlockingShape(const Polygon& poly, int pid);
    void checkAllBlockedEdges(int pid);
    void checkAllMissingEdges(void);
    void adjustContainsWithAdd(const Polygon& poly, const int p_shape);
    void adjustContainsWithDel(const int p_shape);
    void adjustClustersWithAdd(
        const PolygonInterface& poly,
        const int               p_cluster
    );
    void adjustClustersWithDel(const int p_cluster);
    void rerouteAndCallbackConnectors(void);
    void improveCrossings(void);

    ActionInfoList actionList;
    unsigned int   m_largest_assigned_id;
    bool           m_consolidate_actions;
    bool           m_currently_calling_destructors;
    double         m_routing_parameters[lastRoutingParameterMarker];
    bool           m_routing_options[lastRoutingOptionMarker];

    ConnRerouteFlagDelegate m_conn_reroute_flags;
    HyperedgeRerouter       m_hyperedge_rerouter;

    // Progress tracking and transaction cancelling.
    clock_t m_transaction_start_time;
    bool    m_abort_transaction;

    TopologyAddonInterface* m_topology_addon;

    // Overall modes:
    bool m_allows_polyline_routing;
    bool m_allows_orthogonal_routing;

    bool m_static_orthogonal_graph_invalidated;
    bool m_in_crossing_rerouting_stage;

    bool m_settings_changes;

    HyperedgeImprover m_hyperedge_improver;

    DebugHandler* m_debug_handler;
};

}  // namespace avoid

#endif
