///
/// @file AStarPathPrivate.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/AStarPathPrivate.hxx"

namespace avoid {

double AStarPathPrivate::estimatedCost(
    ConnRef*     lineRef,
    const Point* last,
    const Point& curr
) const
{
    double estimate = DBL_MAX;
    COLA_ASSERT(m_cost_targets.size() > 0);

    // Find the minimum cost from the estimates to each of the possible
    // target points from this current point.
    for (size_t i = 0; i < m_cost_targets.size(); ++i)
    {
        double iEstimate = estimatedCostSpecific(
            lineRef,
            last,
            curr,
            m_cost_targets[i],
            m_cost_targets_directions[i]
        );

        // Add on the distance to the real target, otherwise this difference
        // might may make the comparisons unfair if they vary between targets.
        iEstimate += m_cost_targets_displacements[i];

        estimate = std::min(estimate, iEstimate);
    }
    return estimate;
}

void AStarPathPrivate::determineEndPointLocation(
    double   dist,
    VertInf* start,
    VertInf* target,
    VertInf* other,
    int      level
)
{
    COLA_UNUSED(dist);
    COLA_UNUSED(start);
    COLA_UNUSED(level);

    Point        otherPoint = other->point;
    unsigned int thisDirs   = orthogonalDirection(otherPoint, target->point);
    COLA_ASSERT(orthogonalDirectionsCount(thisDirs) > 0);
    double displacement = manhattanDist(otherPoint, target->point);

    m_cost_targets.push_back(other);
    m_cost_targets_directions.push_back(thisDirs);
    m_cost_targets_displacements.push_back(displacement);

#ifdef ESTIMATED_COST_DEBUG
    fprintf(stderr, " - %g %g ", otherPoint.x, otherPoint.y);
    if (manhattanDist(start->point, otherPoint) > dist)
    {
        fprintf(stderr, "far ");
    }
    fprintf(stderr, "%s", (level == 1) ? "--" : "- ");
    printDirections(stderr, thisDirs);
    fprintf(stderr, "\n");
#endif
}

void AStarPathPrivate::search(
    ConnRef* lineRef,
    VertInf* src,
    VertInf* tar,
    VertInf* start
)
{
    ANodeCmp pendingCmp;

    bool isOrthogonal = (lineRef->routingType() == ConnType_Orthogonal);

    if (start == nullptr)
    {
        start = src;
    }

#ifdef DEBUGHANDLER
    if (lineRef->router()->debugHandler())
    {
        lineRef->router()->debugHandler()->beginningSearchWithEndpoints(
            start,
            tar
        );
    }
#endif

    // Find a target point to use for cost estimate for orthogonal routing.
    //
    // If the connectivity is only on the far side we need to estimate to the
    // point on the far side.  Otherwise for orthogonal routing we can explore
    // all the space in between before we pay the extra cost to explore this
    // area.  This is especially true given many orthogonal routes have
    // equivalent costs.
#ifdef ESTIMATED_COST_DEBUG
    fprintf(stderr, "== aStar  %g %g ==\n", tar->point.x, tar->point.y);
#endif
    if (isOrthogonal && tar->id.isConnPt() && !tar->id.isConnCheckpoint())
    {
        // The target is a connector endpoint and the connector is orthogonal.
        double dist = manhattanDist(start->point, tar->point);
        for (EdgeInfList::const_iterator it = tar->orthogVisList.begin();
             it != tar->orthogVisList.end();
             ++it)
        {
            // For each edge from the target endpoint, find the other vertex.
            EdgeInf* edge  = *it;
            VertInf* other = edge->otherVert(tar);
            if (other->id.isConnectionPin())
            {
                // If this is a connection pin we need to do this process
                // another time since the current edge will be a dummy
                // zero-length edge.
                VertInf* replacementTar = other;
                for (EdgeInfList::const_iterator it
                     = replacementTar->orthogVisList.begin();
                     it != replacementTar->orthogVisList.end();
                     ++it)
                {
                    EdgeInf* edge  = *it;
                    VertInf* other = edge->otherVert(replacementTar);
                    if ((other == tar) || (other->point == tar->point))
                    {
                        // Ignore edge we came from, or zero-length edges.
                        continue;
                    }

                    // Determine possible target endpoint directions and
                    // position.
                    determineEndPointLocation(
                        dist,
                        start,
                        replacementTar,
                        other,
                        2
                    );
                }
                continue;
            }

            // Determine possible target endpoint directions and position.
            determineEndPointLocation(dist, start, tar, other, 1);
        }
    }

    if (m_cost_targets.empty())
    {
        m_cost_targets.push_back(tar);
        // For polyline routing, assume target has visibility is all
        // directions for the purpose of cost estimations.
        m_cost_targets_directions.push_back(
            CostDirectionN | CostDirectionE | CostDirectionS | CostDirectionW
        );
        m_cost_targets_displacements.push_back(0.0);
    }

#ifdef ESTIMATED_COST_DEBUG
    fprintf(stderr, "------------\n");
    for (size_t i = 0; i < m_cost_targets.size(); ++i)
    {
        fprintf(
            stderr,
            "== %g %g - ",
            m_cost_targets[i]->point.x,
            m_cost_targets[i]->point.y
        );
        printDirections(stderr, m_cost_targets_directions[i]);
        fprintf(stderr, "\n");
    }
#endif

    double (*dist)(const Point& a, const Point& b)
        = (isOrthogonal) ? manhattanDist : euclideanDist;

    // We need to know the possible endpoints for doing an orthogonal
    // routing optimisation where we only turn when we are heading beside
    // a shape or are in line with a possible endpoint.
    std::vector<Point> endPoints;
    if (isOrthogonal)
    {
        endPoints = lineRef->possibleDstPinPoints();
    }
    endPoints.push_back(tar->point);

    // Heap of PENDING nodes.
    std::vector<ANode*> PENDING;
    PENDING.reserve(1000);

    size_t exploredCount = 0;
    ANode  node, ati;
    ANode* bestNode   = nullptr;  // Temporary bestNode
    bool   bNodeFound = false;    // Flag if node is found in container
    int    timestamp  = 1;

    Router* router = lineRef->router();
    if (router->RubberBandRouting && (start != src))
    {
        COLA_ASSERT(router->IgnoreRegions == true);

        const PolyLine& currRoute = lineRef->route();
        VertInf*        last      = nullptr;
        int             rIndx     = 0;
        while (last != start)
        {
            const Point& pnt   = currRoute.at(rIndx);
            VertIDProps  props = (rIndx > 0) ? 0 : VertID::PROP_ConnPoint;
            VertID       vID(pnt.id, pnt.vn, props);

#ifdef PATHDEBUG
            db_printf("/// %d %d\n", pnt.id, pnt.vn);
#endif
            VertInf* curr = router->vertices.getVertexByID(vID);
            COLA_ASSERT(curr != nullptr);

            node = ANode(curr, timestamp++);
            if (!last)
            {
                node.inf = src;
                node.g   = 0;
                node.h   = estimatedCost(lineRef, nullptr, node.inf->point);

                node.f = node.g + node.h;
            }
            else
            {
                double edgeDist = dist(bestNode->inf->point, curr->point);

                node.g = bestNode->g
                       + cost(
                             lineRef,
                             edgeDist,
                             bestNode->inf,
                             node.inf,
                             bestNode->prevNode
                       );

                // Calculate the Heuristic.
                node.h = estimatedCost(
                    lineRef,
                    &(bestNode->inf->point),
                    node.inf->point
                );

                // The A* formula
                node.f = node.g + node.h;

                // Point parent to last bestNode
                node.prevNode = bestNode;
            }

            if (curr != start)
            {
                bool addToPending = false;
                bestNode          = newANode(node, addToPending);
                bestNode->inf->aStarDoneNodes.push_back(bestNode);
                ++exploredCount;
            }
            else
            {
                ANode* newNode = newANode(node);
                PENDING.push_back(newNode);
            }

            rIndx++;
            last = curr;
        }
    }
    else
    {
        if (start->pathNext)
        {
            // If we are doing checkpoint routing and have already done one
            // path, then we have an existing segment to consider for the
            // cost of the  choice from the start node, so we add a dummy
            // nodes as if they were already in the Done set.  This causes
            // us to first search in a collinear direction from the previous
            // segment.
            bool addToPending = false;
            bestNode
                = newANode(ANode(start->pathNext, timestamp++), addToPending);
            bestNode->inf->aStarDoneNodes.push_back(bestNode);
            ++exploredCount;
        }

        // Create the start node
        node          = ANode(src, timestamp++);
        node.g        = 0;
        node.h        = estimatedCost(lineRef, nullptr, node.inf->point);
        node.f        = node.g + node.h;
        // Set a nullptr parent, so cost function knows this is the first
        // segment.
        node.prevNode = bestNode;

        // Populate the PENDING container with the first location
        ANode* newNode = newANode(node);
        PENDING.push_back(newNode);
    }

    tar->pathNext = nullptr;

    // Create a heap from PENDING for sorting
    using std::make_heap;
    using std::pop_heap;
    using std::push_heap;
    make_heap(PENDING.begin(), PENDING.end(), pendingCmp);

    // Continue until the queue is empty.
    while (!PENDING.empty())
    {
        TIMER_VAR_ADD(router, 0, 1);
        // Set the Node with lowest f value to BESTNODE.
        // Since the ANode operator< is reversed, the head of the
        // heap is the node with the lowest f value.
        bestNode             = PENDING.front();
        VertInf* bestNodeInf = bestNode->inf;

#ifdef DEBUGHANDLER
        if (router->debugHandler())
        {
            PolyLine currentSearchPath;

            ANode* curr = bestNode;
            while (curr)
            {
                currentSearchPath.ps.push_back(curr->inf->point);
                curr = curr->prevNode;
            }
            router->debugHandler()->updateCurrentSearchPath(currentSearchPath);
        }
#endif

        // Remove this node from the aStarPendingList
        std::list<ANode*>::iterator finishIt
            = bestNodeInf->aStarPendingNodes.end();
        for (std::list<ANode*>::iterator currInd
             = bestNodeInf->aStarPendingNodes.begin();
             currInd != finishIt;
             ++currInd)
        {
            if (*currInd == bestNode)
            {
                bestNodeInf->aStarPendingNodes.erase(currInd);
                break;
            }
        }

        // Pop off the heap.  Actually this moves the
        // far left value to the far right.  The node
        // is not actually removed since the pop is to
        // the heap and not the container.
        pop_heap(PENDING.begin(), PENDING.end(), pendingCmp);
        // Remove node from right (the value we pop_heap'd)
        PENDING.pop_back();

        // Add the bestNode into the Done set.
        bestNodeInf->aStarDoneNodes.push_back(bestNode);
        ++exploredCount;

        VertInf* prevInf
            = (bestNode->prevNode) ? bestNode->prevNode->inf : nullptr;
#ifdef ASTAR_DEBUG
        db_printf("Considering... ");
        db_printf(" %g %g  ", bestNodeInf->point.x, bestNodeInf->point.y);
        bestNodeInf->id.db_print();
        db_printf(" - g: %3.1f h: %3.1f back: ", bestNode->g, bestNode->h);
        if (prevInf)
        {
            db_printf(" %g %g", prevInf->point.x, prevInf->point.y);
            // prevInf->id.db_print();
        }
        db_printf("\n");
#endif

        if (bestNodeInf == tar)
        {
            TIMER_VAR_ADD(router, 1, PENDING.size());
            // This node is our goal.
#ifdef ASTAR_DEBUG
            db_printf(
                "LINE %10d  Steps: %4d  Cost: %g\n",
                lineRef->id(),
                (int)exploredCount,
                bestNode->f
            );
#endif

            // Correct all the pathNext pointers.
            for (ANode* curr = bestNode; curr->prevNode; curr = curr->prevNode)
            {
#ifdef ASTAR_DEBUG
                db_printf(
                    "[%.12f, %.12f]\n",
                    curr->inf->point.x,
                    curr->inf->point.y
                );
#endif
                curr->inf->pathNext = curr->prevNode->inf;
            }
#ifdef ASTAR_DEBUG
            db_printf("\n");
#endif

            // Exit from the search
            break;
        }

        // Check adjacent points in graph and add them to the queue.
        EdgeInfList& visList = (!isOrthogonal) ? bestNodeInf->visList
                                               : bestNodeInf->orthogVisList;
        if (isOrthogonal)
        {
            // We would like to explore in a structured way,
            // so sort the points in the visList...
            CmpVisEdgeRotation compare(prevInf);
            visList.sort(compare);
        }
        EdgeInfList::const_iterator finish = visList.end();
        for (EdgeInfList::const_iterator edge = visList.begin(); edge != finish;
             ++edge)
        {
            if ((*edge)->isDisabled())
            {
                // Skip disabled edges.
                continue;
            }

            node = ANode((*edge)->otherVert(bestNodeInf), timestamp++);

            // Set the index to the previous ANode that we reached
            // this ANode via.
            node.prevNode = bestNode;

            VertInf* prevInf
                = (bestNode->prevNode) ? bestNode->prevNode->inf : nullptr;

            // Don't bother looking at the segment we just arrived along.
            if (prevInf && (prevInf == node.inf))
            {
                continue;
            }
            if (node.inf->id.isConnectionPin()
                && !node.inf->id.isConnCheckpoint())
            {
                if (!((bestNodeInf == lineRef->src())
                      && lineRef->src()->id.isDummyPinHelper())
                    && !(
                        node.inf->hasNeighbour(lineRef->dst(), isOrthogonal)
                        && lineRef->dst()->id.isDummyPinHelper()
                    ))
                {
                    // Don't check connection pins if they don't have the
                    // target vertex as a direct neighbour, or are directly
                    // leaving the source vertex.
                    continue;
                }
            }
            else if (node.inf->id.isConnPt())
            {
                if ((node.inf != tar))
                {
                    // Don't check connector endpoints vertices unless they
                    // are the target endpoint.
                    continue;
                }
            }

            if (isOrthogonal && !(*edge)->isDummyConnection())
            {
                // Orthogonal routing optimisation.
                // Skip the edges that don't lead to shape edges, or the
                // connection point we are looking for.  Though allow them
                // if we haven't yet turned from the source point, since it
                // may be a free-floating endpoint with directional visibility.
                // Also, don't check if the previous point was a dummy for a
                // connection pin and this happens to be placed diagonally
                // from here, i.e., when both of notInline{X,Y} are true.
                Point& bestPt = bestNodeInf->point;
                Point& nextPt = node.inf->point;

                bool notInlineX = prevInf && (prevInf->point.x != bestPt.x);
                bool notInlineY = prevInf && (prevInf->point.y != bestPt.y);
                if ((bestPt.x == nextPt.x) && notInlineX && !notInlineY
                    && (bestPt[YDIM] != src->point[YDIM]))
                {
                    if (nextPt.y < bestPt.y)
                    {
                        if (!(bestNodeInf->orthogVisPropFlags & YL_EDGE)
                            && !pointAlignedWithOneOf(bestPt, endPoints, XDIM))
                        {
                            continue;
                        }
                    }
                    else if (nextPt.y > bestPt.y)
                    {
                        if (!(bestNodeInf->orthogVisPropFlags & YH_EDGE)
                            && !pointAlignedWithOneOf(bestPt, endPoints, XDIM))
                        {
                            continue;
                        }
                    }
                }
                if ((bestPt.y == nextPt.y) && notInlineY && !notInlineX
                    && (bestPt[XDIM] != src->point[XDIM]))
                {
                    if (nextPt.x < bestPt.x)
                    {
                        if (!(bestNodeInf->orthogVisPropFlags & XL_EDGE)
                            && !pointAlignedWithOneOf(bestPt, endPoints, YDIM))
                        {
                            continue;
                        }
                    }
                    else if (nextPt.x > bestPt.x)
                    {
                        if (!(bestNodeInf->orthogVisPropFlags & XH_EDGE)
                            && !pointAlignedWithOneOf(bestPt, endPoints, YDIM))
                        {
                            continue;
                        }
                    }
                }
            }

            double edgeDist = (*edge)->getDist();

            if (edgeDist == 0)
            {
                continue;
            }

            if (!isOrthogonal && (!router->RubberBandRouting || (start == src))
                && (validateBendPoint(prevInf, bestNodeInf, node.inf) == false))
            {
                // The bendpoint is not valid, i.e., is a zigzag corner, so...
                continue;
                // For RubberBand routing we want to allow these routes and
                // unwind them later, otherwise instead or unwinding, paths
                // can go the *really* long way round.
            }

            // Figure out if we are at one of the cost targets.
            bool atCostTarget = false;
            for (size_t i = 0; i < m_cost_targets.size(); ++i)
            {
                if (bestNode->inf == m_cost_targets[i])

                {
                    atCostTarget = true;
                    break;
                }
            }

            if (atCostTarget
                && (node.inf->id.isConnectionPin() || (node.inf == tar)))
            {
                // This is a point on the side of an obstacle that connects
                // to the target or a connection pin.  It should have no
                // further cost and the heuristic should be zero.
                node.g = bestNode->g;
                node.h = 0;
            }
            else
            {
                if (node.inf == tar)
                {
                    // We've reached the target.  The heuristic should be zero.
                    node.h = 0;
                }
                else
                {
                    // Otherwise, calculate the heuristic value.
                    node.h = estimatedCost(
                        lineRef,
                        &(bestNodeInf->point),
                        node.inf->point
                    );
                }

                if (node.inf->id.isDummyPinHelper())
                {
                    // This is connecting to a connection pin helper vertex.
                    // There should be no additional cost for this step.
                    node.g = bestNode->g;
                }
                else
                {
                    // Otherwise, calculate the cost of this step.
                    node.g = bestNode->g
                           + cost(
                                 lineRef,
                                 edgeDist,
                                 bestNodeInf,
                                 node.inf,
                                 bestNode->prevNode
                           );
                }
            }

            // The A* formula
            node.f = node.g + node.h;

#ifdef ASTAR_DEBUG
            db_printf(
                "-- Adding: %g %g  ",
                node.inf->point.x,
                node.inf->point.y
            );
            node.inf->id.db_print();
            db_printf(" - g: %3.1f h: %3.1f \n", node.g, node.h);
#endif

            bNodeFound = false;

            // Check to see if already on PENDING
            std::list<ANode*>::const_iterator finish
                = node.inf->aStarPendingNodes.end();
            for (std::list<ANode*>::const_iterator currInd
                 = node.inf->aStarPendingNodes.begin();
                 currInd != finish;
                 ++currInd)
            {
                ati = **currInd;
                // The (node.prevNode == ati.prevNode) is redundant, but may
                // save checking the mosre costly prevNode->inf test if the
                // Nodes are the same.
                if ((node.inf == ati.inf)
                    && ((node.prevNode == ati.prevNode)
                        || (node.prevNode->inf == ati.prevNode->inf)))
                {
                    // If already on PENDING
                    if (node.g < ati.g)
                    {
                        // Replace the existing node in PENDING
                        **currInd = node;
                        make_heap(PENDING.begin(), PENDING.end(), pendingCmp);
                    }
                    bNodeFound = true;
                    break;
                }
            }
            if (!bNodeFound)  // If Node NOT found on PENDING
            {
                // Check to see if it is already in the Done set for this
                // vertex.
                for (std::list<ANode*>::const_iterator currInd
                     = node.inf->aStarDoneNodes.begin();
                     currInd != node.inf->aStarDoneNodes.end();
                     ++currInd)
                {
                    ati = **currInd;
                    // The (node.prevNode == ati.prevNode) is redundant, but may
                    // save checking the mosre costly prevNode->inf test if the
                    // Nodes are the same.
                    if ((node.inf == ati.inf) && ati.prevNode
                        && ((node.prevNode == ati.prevNode)
                            || (node.prevNode->inf == ati.prevNode->inf)))
                    {
                        // COLA_ASSERT(node.g >= (ati.g - 10e-10));
                        //  This node is already in the Done set and the
                        //  current node also has a higher g-value, so we
                        //  don't need to consider this node.
                        bNodeFound = true;
                        break;
                    }
                }
            }

            if (!bNodeFound)  // If Node NOT in either Pending or Done.
            {
                // Push NewNode onto PENDING
                ANode* newNode = newANode(node);
                PENDING.push_back(newNode);
                // Push NewNode onto heap
                push_heap(PENDING.begin(), PENDING.end(), pendingCmp);

#if 0
                using std::cout; using std::endl;
                // Display PENDING container (For Debugging)
                cout << "PENDING:   ";
                for (unsigned int i = 0; i < PENDING.size(); i++)
                {
                    cout << PENDING[i]->g << "," << PENDING[i]->h << ",";
                    cout << PENDING[i]->inf << "," << PENDING[i]->pp << "  ";
                }
                cout << endl << endl;
#endif
            }
        }
    }

    // Cleanup lists used to store Done and Pending sets for each vertex.
    VertInf* endVert = router->vertices.end();
    for (VertInf* k = router->vertices.connsBegin(); k != endVert;
         k          = k->lstNext)
    {
        k->aStarDoneNodes.clear();
        k->aStarPendingNodes.clear();
    }
}

}  // namespace avoid
