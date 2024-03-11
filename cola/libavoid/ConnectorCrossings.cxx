///
/// @file ConnectorCrossings.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/ConnectorCrossings.hxx"

namespace avoid {

ConnectorCrossings::ConnectorCrossings(
    avoid::Polygon& poly,
    bool            polyIsConn,
    Avoid::Polygon& conn,
    ConnRef*        polyConnRef,
    ConnRef*        connConnRef
)
    : poly(poly)
    , polyIsConn(polyIsConn)
    , conn(conn)
    , checkForBranchingSegments(false)
    , polyConnRef(polyConnRef)
    , connConnRef(connConnRef)
    , crossingPoints(nullptr)
    , pointOrders(nullptr)
    , sharedPaths(nullptr)
{
}

void ConnectorCrossings::clear(void)
{
    crossingCount               = 0;
    crossingFlags               = CROSSING_NONE;
    firstSharedPathAtEndLength  = DBL_MAX;
    secondSharedPathAtEndLength = DBL_MAX;
}

// Works out if the segment conn[cIndex-1]--conn[cIndex] really crosses poly.
// This does not not count non-crossing shared paths as crossings.
// poly can be either a connector (polyIsConn = true) or a cluster
// boundary (polyIsConn = false).
//
void ConnectorCrossings::countForSegment(size_t cIndex, const bool finalSegment)
{
    clear();

    bool polyIsOrthogonal
        = (polyConnRef && (polyConnRef->routingType() == ConnType_Orthogonal));
    bool connIsOrthogonal
        = (connConnRef && (connConnRef->routingType() == ConnType_Orthogonal));

    // Fixed routes will not have segment breaks at possible crossings.
    bool polyIsFixed = (polyConnRef && polyConnRef->hasFixedRoute());
    bool connIsFixed = (connConnRef && connConnRef->hasFixedRoute());

    // We need to split apart connectors at potential crossing points if
    // either has a fixed route or it is a polyline connector.  This is not
    // needed for orthogonal connectors where crossings occur at vertices
    // in visibility graph and on the raw connector routes.
    if (checkForBranchingSegments || polyIsFixed || connIsFixed
        || !polyIsOrthogonal || !connIsOrthogonal)
    {
        double epsilon   = std::numeric_limits<double>::epsilon();
        size_t conn_pn   = conn.size();
        // XXX When doing the pointOnLine test we allow the points to be
        // slightly non-collinear.  This addresses a problem with clustered
        // routing where connectors could otherwise route cheaply through
        // shape corners that were not quite on the cluster boundary, but
        // reported to be on there by the line segment intersection code,
        // which I suspect is not numerically accurate enough.  This occurred
        // for points that only differed by about 10^-12 in the y-dimension.
        double tolerance = (!polyIsConn) ? epsilon : 0.0;
        splitBranchingSegments(poly, polyIsConn, conn, tolerance);
        // cIndex is going to be the last, so take into account added points.
        cIndex += (conn.size() - conn_pn);
    }
    COLA_ASSERT(cIndex >= 1);
    COLA_ASSERT(cIndex < conn.size());

    size_t poly_size = poly.size();

    Avoid::Point& a1 = conn.ps[cIndex - 1];
    Avoid::Point& a2 = conn.ps[cIndex];
    // db_printf("a1: %g %g\n", a1.x, a1.y);
    // db_printf("a2: %g %g\n", a2.x, a2.y);

    // Allocate arrays for computing shared paths.
    // Don't use dynamic array due to portablity issues.
    size_t         max_path_size = std::min(poly_size, conn.size());
    Avoid::Point** c_path        = new Avoid::Point*[max_path_size];
    Avoid::Point** p_path        = new Avoid::Point*[max_path_size];
    size_t         size          = 0;

    for (size_t j = ((polyIsConn) ? 1 : 0); j < poly_size; ++j)
    {
        Avoid::Point& b1 = poly.ps[(j - 1 + poly_size) % poly_size];
        Avoid::Point& b2 = poly.ps[j];
        // db_printf("b1: %g %g\n", b1.x, b1.y);
        // db_printf("b2: %g %g\n", b2.x, b2.y);

        size = 0;

        bool converging = false;

        const bool a1_eq_b1 = (a1 == b1);
        const bool a2_eq_b1 = (a2 == b1);
        const bool a2_eq_b2 = (a2 == b2);
        const bool a1_eq_b2 = (a1 == b2);

        if ((a1_eq_b1 && a2_eq_b2) || (a2_eq_b1 && a1_eq_b2))
        {
            if (finalSegment)
            {
                converging = true;
            }
            else
            {
                // Route along same segment: no penalty.  We detect
                // crossovers when we see the segments diverge.
                continue;
            }
        }
        else if (a2_eq_b1 || a2_eq_b2 || a1_eq_b2)
        {
            // Each crossing that is at a vertex in the
            // visibility graph gets noticed four times.
            // We ignore three of these cases.
            // This also catches the case of a shared path,
            // but this is one that terminates at a common
            // endpoint, so we don't care about it.
            continue;
        }

        if (a1_eq_b1 || converging)
        {
            if (!converging)
            {
                if (polyIsConn && (j == 1))
                {
                    // Can't be the end of a shared path or crossing path
                    // since the common point is the first point of the
                    // connector path.  This is not a shared path at all.
                    continue;
                }

                Avoid::Point& b0 = poly.ps[(j - 2 + poly_size) % poly_size];
                // The segments share an endpoint -- a1==b1.
                if (a2 == b0)
                {
                    // a2 is not a split, continue.
                    continue;
                }
            }

            // If here and not converging, then we know that a2 != b2
            // And a2 and its pair in b are a split.
            COLA_ASSERT(converging || !a2_eq_b2);

            bool shared_path = false;

            // Initial values here don't matter. They are only used after
            // being set to sensible values, but we set them to stop a MSVC
            // warning.
            bool p_dir_back;
            int  p_dir   = 0;
            int  trace_c = 0;
            int  trace_p = 0;

            if (converging)
            {
                // Determine direction we have to look through
                // the points of connector b.
                p_dir_back = a2_eq_b2 ? true : false;
                p_dir      = p_dir_back ? -1 : 1;
                trace_c    = (int)cIndex;
                trace_p    = (int)j;
                if (!p_dir_back)
                {
                    if (finalSegment)
                    {
                        trace_p--;
                    }
                    else
                    {
                        trace_c--;
                    }
                }

                shared_path = true;
            }
            else if (cIndex >= 2)
            {
                Avoid::Point& b0 = poly.ps[(j - 2 + poly_size) % poly_size];
                Avoid::Point& a0 = conn.ps[cIndex - 2];

                // db_printf("a0: %g %g\n", a0.x, a0.y);
                // db_printf("b0: %g %g\n", b0.x, b0.y);

                if ((a0 == b2) || (a0 == b0))
                {
                    // Determine direction we have to look through
                    // the points of connector b.
                    p_dir_back = (a0 == b0) ? true : false;
                    p_dir      = p_dir_back ? -1 : 1;
                    trace_c    = (int)cIndex;
                    trace_p    = (int)(p_dir_back ? j : j - 2);

                    shared_path = true;
                }
            }

            if (shared_path)
            {
                crossingFlags |= CROSSING_SHARES_PATH;
                // Shouldn't be here if p_dir is still equal to zero.
                COLA_ASSERT(p_dir != 0);

                // Build the shared path, including the diverging points at
                // each end if the connector does not end at a common point.
                while ((trace_c >= 0)
                       && (!polyIsConn
                           || ((trace_p >= 0) && (trace_p < (int)poly_size))))
                {
                    // If poly is a cluster boundary, then it is a closed
                    // poly-line and so it wraps around.
                    size_t index_p
                        = (size_t)((trace_p + (2 * poly_size)) % poly_size);
                    size_t index_c = (size_t)trace_c;
                    c_path[size]   = &conn.ps[index_c];
                    p_path[size]   = &poly.ps[index_p];
                    ++size;
                    if ((size > 1) && (conn.ps[index_c] != poly.ps[index_p]))
                    {
                        // Points don't match, so break out of loop.
                        break;
                    }
                    trace_c--;
                    trace_p += p_dir;
                }

                // Are there diverging points at the ends of the shared path.
                bool front_same = (*(c_path[0]) == *(p_path[0]));
                bool back_same  = (*(c_path[size - 1]) == *(p_path[size - 1]));

                // Determine if the shared path originates at a junction.
                bool terminatesAtJunction = false;
                if (polyConnRef && connConnRef && (front_same || back_same))
                {
                    // To do this we find the two ConnEnds at the common
                    // end of the shared path.  Then check if they are
                    // attached to a junction and it is the same one.
                    std::pair<ConnEnd, ConnEnd> connEnds
                        = connConnRef->endpointConnEnds();
                    JunctionRef* connJunction = nullptr;

                    std::pair<ConnEnd, ConnEnd> polyEnds
                        = polyConnRef->endpointConnEnds();
                    JunctionRef* polyJunction = nullptr;

                    // The front of the c_path corresponds to destination
                    // of the connector.
                    connJunction   = (front_same) ? connEnds.second.junction()
                                                  : connEnds.first.junction();
                    bool use_first = back_same;
                    if (p_dir_back)
                    {
                        // Reversed, so use opposite.
                        use_first = !use_first;
                    }
                    // The front of the p_path corresponds to destination
                    // of the connector.
                    polyJunction = (use_first) ? polyEnds.second.junction()
                                               : polyEnds.first.junction();

                    terminatesAtJunction
                        = (connJunction && (connJunction == polyJunction));
                }

                if (sharedPaths)
                {
                    // Store a copy of the shared path
                    size_t start = (front_same) ? 0 : 1;
                    size_t limit = size - ((back_same) ? 0 : 1);

                    PointList sPath(limit - start);
                    for (size_t i = start; i < limit; ++i)
                    {
                        sPath[i - start] = *(c_path[i]);
                    }
                    sharedPaths->push_back(sPath);
                }

                // Check to see if these share a fixed segment.
                if (polyIsOrthogonal && connIsOrthogonal)
                {
                    size_t startPt = (front_same) ? 0 : 1;
                    size_t endPt   = size - ((back_same) ? 1 : 2);
                    for (size_t dim = 0; dim < 2; ++dim)
                    {
                        if ((*c_path[startPt])[dim] == (*c_path[endPt])[dim])
                        {
                            double pos = (*c_path[startPt])[dim];
                            // See if this is inline with either the start
                            // or end point of both connectors.  We don't
                            // count them as crossing if they originate at a
                            // junction and are part of the same hyperedge.
                            if (((pos == poly.ps[0][dim])
                                 || (pos == poly.ps[poly_size - 1][dim]))
                                && ((pos == conn.ps[0][dim])
                                    || (pos == conn.ps[cIndex][dim]))
                                && (terminatesAtJunction == false))
                            {
                                crossingFlags |= CROSSING_SHARES_FIXED_SEGMENT;
                            }
                        }
                    }

                    if (!front_same && !back_same)
                    {
                        // Find overlapping segments that are constrained by
                        // the fact that both the adjoining segments are fixed
                        // in the other dimension, i.e.,
                        //
                        // X------++---X
                        //        ||
                        //        ||
                        //    X---++------X
                        //
                        // In the example above, altDim is X, and dim is Y.
                        //

                        // For each dimension...
                        for (size_t dim = 0; dim < 2; ++dim)
                        {
                            size_t end    = size - 1;
                            size_t altDim = (dim + 1) % 2;
                            // If segment is in this dimension...
                            if ((*c_path[1])[altDim]
                                == (*c_path[end - 1])[altDim])
                            {
                                double posBeg = (*c_path[1])[dim];
                                double posEnd = (*c_path[end - 1])[dim];
                                // If both segment ends diverge at
                                // right-angles...
                                if ((posBeg == (*c_path[0])[dim])
                                    && (posBeg == (*p_path[0])[dim])
                                    && (posEnd == (*c_path[end])[dim])
                                    && (posEnd == (*p_path[end])[dim]))
                                {
                                    // and these segments are inline with the
                                    // conn and path ends themselves...
                                    if (posInlineWithConnEndSegs(
                                            posBeg,
                                            dim,
                                            conn,
                                            poly
                                        )
                                        && posInlineWithConnEndSegs(
                                            posEnd,
                                            dim,
                                            conn,
                                            poly
                                        ))
                                    {
                                        // If all endpoints branch at right
                                        // angles, then penalise this since it
                                        // is a segment will will not be able to
                                        // nudge apart without introducing fixed
                                        // segment crossings.
                                        crossingFlags
                                            |= CROSSING_SHARES_FIXED_SEGMENT;
                                    }
                                }
                            }
                        }
                    }

#if 0
                    // XXX: What is this code for?  It is pretty much 
                    // incomprehensible and also causes one of the test
                    // cases to fail.  
                    //
                    if (!front_same && !back_same)
                    {
                        for (size_t dim = 0; dim < 2; ++dim)
                        {
                            size_t altDim = (dim + 1) % 2;
                            if ((*c_path[1])[altDim] == (*c_path[1])[altDim])
                            {
                                size_t n = c_path.size();
                                double yPosB = (*c_path[1])[dim];
                                if ( (yPosB == (*c_path[0])[dim]) && 
                                        (yPosB == (*p_path[0])[dim]) )
                                {
                                    crossingFlags |= 
                                            CROSSING_SHARES_FIXED_SEGMENT;
                                }

                                double yPosE = (*c_path[n - 2])[dim];
                                if ( (yPosE == (*c_path[n - 1])[dim]) && 
                                        (yPosE == (*p_path[n - 1])[dim]) )
                                {
                                    crossingFlags |= 
                                            CROSSING_SHARES_FIXED_SEGMENT;
                                }
                            }
                        }
                    }
#endif
                }

                // {start,end}CornerSide specifies which side of conn the
                // poly path enters and leaves.
                int startCornerSide = 1;
                int endCornerSide   = 1;

                bool reversed = false;
                if (!front_same)
                {
                    // If there is a divergence at the beginning,
                    // then order the shared path based on this.
                    startCornerSide = Avoid::cornerSide(
                        *c_path[0],
                        *c_path[1],
                        *c_path[2],
                        *p_path[0]
                    );
                }
                if (!back_same)
                {
                    // If there is a divergence at the end of the path,
                    // then order the shared path based on this.
                    endCornerSide = Avoid::cornerSide(
                        *c_path[size - 3],
                        *c_path[size - 2],
                        *c_path[size - 1],
                        *p_path[size - 1]
                    );
                }
                else
                {
                    endCornerSide = startCornerSide;
                }
                if (front_same)
                {
                    startCornerSide = endCornerSide;
                }

                if (endCornerSide != startCornerSide)
                {
                    // Mark that the shared path crosses.
                    // db_printf("shared path crosses.\n");
                    crossingCount += 1;
                    if (crossingPoints)
                    {
                        crossingPoints->insert(*c_path[1]);
                    }
                }

                if (front_same || back_same)
                {
                    crossingFlags |= CROSSING_SHARES_PATH_AT_END;

                    // Reduce the cost of paths that stay straight after
                    // the split, and make this length available so that the
                    // paths can be ordered during the improveCrossings()
                    // step and the straight (usually better) paths will be
                    // left alone while the router attempts to find better
                    // paths for the others.
                    double straightModifier    = 200;
                    firstSharedPathAtEndLength = secondSharedPathAtEndLength
                        = pathLength(c_path, p_path, size);
                    if (back_same && (size > 2))
                    {
                        if (vecDir(*p_path[0], *p_path[1], *p_path[2]) == 0)
                        {
                            firstSharedPathAtEndLength -= straightModifier;
                        }

                        if (vecDir(*c_path[0], *c_path[1], *c_path[2]) == 0)
                        {
                            secondSharedPathAtEndLength -= straightModifier;
                        }
                    }
                    else if (front_same && (size > 2))
                    {
                        if (vecDir(
                                *p_path[size - 3],
                                *p_path[size - 2],
                                *p_path[size - 1]
                            )
                            == 0)
                        {
                            firstSharedPathAtEndLength -= straightModifier;
                        }

                        if (vecDir(
                                *c_path[size - 3],
                                *c_path[size - 2],
                                *c_path[size - 1]
                            )
                            == 0)
                        {
                            secondSharedPathAtEndLength -= straightModifier;
                        }
                    }
                }
                else if (polyIsOrthogonal && connIsOrthogonal)
                {
                    int cStartDir = vecDir(*c_path[0], *c_path[1], *c_path[2]);
                    int pStartDir = vecDir(*p_path[0], *p_path[1], *p_path[2]);
                    if ((cStartDir != 0) && (cStartDir == -pStartDir))
                    {
                        // The start segments diverge at 180 degrees to each
                        // other.  So order based on not introducing overlap
                        // of the diverging segments when these are nudged
                        // apart.
                        startCornerSide = -cStartDir;
                    }
                    else
                    {
                        int cEndDir = vecDir(
                            *c_path[size - 3],
                            *c_path[size - 2],
                            *c_path[size - 1]
                        );
                        int pEndDir = vecDir(
                            *p_path[size - 3],
                            *p_path[size - 2],
                            *p_path[size - 1]
                        );
                        if ((cEndDir != 0) && (cEndDir == -pEndDir))
                        {
                            // The end segments diverge at 180 degrees to
                            // each other.  So order based on not introducing
                            // overlap of the diverging segments when these
                            // are nudged apart.
                            startCornerSide = -cEndDir;
                        }
                    }
                }

#if 0
                int prevTurnDir = 0;
                if (pointOrders)
                {
                    // Return the ordering for the shared path.
                    COLA_ASSERT(c_path.size() > 0 || back_same);
                    size_t adj_size = (c_path.size() - ((back_same) ? 0 : 1));
                    for (size_t i = (front_same) ? 0 : 1; i < adj_size; ++i)
                    {
                        Avoid::Point& an = *(c_path[i]);
                        Avoid::Point& bn = *(p_path[i]);
                        int currTurnDir = ((i > 0) && (i < (adj_size - 1))) ?  
                                vecDir(*c_path[i - 1], an,
                                       *c_path[i + 1]) : 0;
                        VertID vID(an.id, true, an.vn);
                        if ( (currTurnDir == (-1 * prevTurnDir)) &&
                                (currTurnDir != 0) && (prevTurnDir != 0) )
                        {
                            // The connector turns the opposite way around 
                            // this shape as the previous bend on the path,
                            // so reverse the order so that the inner path
                            // become the outer path and vice versa.
                            reversed = !reversed;
                        }
                        bool orderSwapped = (*pointOrders)[an].addOrderedPoints(
                                &bn, &an, reversed);
                        if (orderSwapped)
                        {
                            // Reverse the order for later points.
                            reversed = !reversed;
                        }
                        prevTurnDir = currTurnDir;
                    }
                }
#endif
                if (pointOrders)
                {
                    reversed       = false;
                    size_t startPt = (front_same) ? 0 : 1;

                    // Orthogonal should always have at least one segment.
                    COLA_ASSERT(size > (startPt + 1));

                    if (startCornerSide > 0)
                    {
                        reversed = !reversed;
                    }

                    int prevDir = 0;
                    // Return the ordering for the shared path.
                    COLA_ASSERT(size > 0 || back_same);
                    size_t adj_size = (size - ((back_same) ? 0 : 1));
                    for (size_t i = startPt; i < adj_size; ++i)
                    {
                        Avoid::Point& an = *(c_path[i]);
                        Avoid::Point& bn = *(p_path[i]);
                        COLA_ASSERT(an == bn);

                        if (i > startPt)
                        {
                            Avoid::Point& ap = *(c_path[i - 1]);
                            Avoid::Point& bp = *(p_path[i - 1]);

                            int thisDir = segDir(ap, an);
                            if (prevDir == 0)
                            {
                                if (thisDir > 0)
                                {
                                    reversed = !reversed;
                                }
                            }
                            else if (thisDir != prevDir)
                            {
                                reversed = !reversed;
                            }

                            int orientation = (ap.x == an.x) ? 0 : 1;
                            // printf("prevOri %d\n", prevOrientation);
                            // printf("1: %X, %X\n", (int) &(bn), (int) &(an));
                            (*pointOrders)[an].addOrderedPoints(
                                orientation,
                                std::make_pair(&bn, polyConnRef),
                                std::make_pair(&an, connConnRef),
                                reversed
                            );
                            COLA_ASSERT(ap == bp);
                            // printf("2: %X, %X\n", (int) &bp, (int) &ap);
                            (*pointOrders)[ap].addOrderedPoints(
                                orientation,
                                std::make_pair(&bp, polyConnRef),
                                std::make_pair(&ap, connConnRef),
                                reversed
                            );
                            prevDir = thisDir;
                        }
                    }
                }
#if 0
                    int ymod = -1;
                    if ((id.vn == 1) || (id.vn == 2))
                    {
                        // bottom.
                        ymod = +1;
                    }
                    
                    int xmod = -1;
                    if ((id.vn == 0) || (id.vn == 1))
                    {
                        // right.
                        xmod = +1;
                    }
                    if(id.vn > 3)
                    {
                        xmod = ymod = 0;
                        if (id.vn == 4)
                        {
                            // right.
                            xmod = +1;
                        }
                        else if (id.vn == 5)
                        {
                            // bottom.
                            ymod = +1;
                        }
                        else if (id.vn == 6)
                        {
                            // left.
                            xmod = -1;
                        }
                        else if (id.vn == 7)
                        {
                            // top.
                            ymod = -1;
                        }
                    }
#endif

                crossingFlags |= CROSSING_TOUCHES;
            }
            else if (cIndex >= 2)
            {
                // The connectors cross or touch at this point.
                // db_printf("Cross or touch at point... \n");

                // Crossing shouldn't be at an endpoint.
                COLA_ASSERT(cIndex >= 2);
                COLA_ASSERT(!polyIsConn || (j >= 2));

                Avoid::Point& b0 = poly.ps[(j - 2 + poly_size) % poly_size];
                Avoid::Point& a0 = conn.ps[cIndex - 2];

                int side1 = Avoid::cornerSide(a0, a1, a2, b0);
                int side2 = Avoid::cornerSide(a0, a1, a2, b2);
                if (side1 != side2)
                {
                    // The connectors cross at this point.
                    // db_printf("cross.\n");
                    crossingCount += 1;
                    if (crossingPoints)
                    {
                        crossingPoints->insert(a1);
                    }
                }

                crossingFlags |= CROSSING_TOUCHES;
                if (pointOrders)
                {
                    if (polyIsOrthogonal && connIsOrthogonal)
                    {
                        // Orthogonal case:
                        // Just order based on which comes from the left and
                        // top in each dimension because this can only be two
                        // L-shaped segments touching at the bend.
                        bool reversedX = ((a0.x < a1.x) || (a2.x < a1.x));
                        bool reversedY = ((a0.y < a1.y) || (a2.y < a1.y));
                        // XXX: Why do we need to invert the reversed values
                        //      here?  Are they wrong for orthogonal points
                        //      in the other places?
                        (*pointOrders)[b1].addOrderedPoints(
                            0,
                            std::make_pair(&b1, polyConnRef),
                            std::make_pair(&a1, connConnRef),
                            !reversedX
                        );
                        (*pointOrders)[b1].addOrderedPoints(
                            1,
                            std::make_pair(&b1, polyConnRef),
                            std::make_pair(&a1, connConnRef),
                            !reversedY
                        );
                    }
#if 0
// Unused code.
                    else
                    {
                        int turnDirA = vecDir(a0, a1, a2); 
                        int turnDirB = vecDir(b0, b1, b2); 
                        bool reversed = (side1 != -turnDirA); 
                        if (side1 != side2) 
                        { 
                            // Interesting case where a connector routes round 
                            // the edge of a shape and intersects a connector 
                            // which is connected to a port on the edge of the 
                            // shape. 
                            if (turnDirA == 0) 
                            { 
                                // We'll make B the outer by preference,  
                                // because the points of A are collinear. 
                                reversed = false; 
                            } 
                            else if (turnDirB == 0) 
                            { 
                                reversed = true; 
                            } 
                            // TODO COLA_ASSERT((turnDirB != 0) || 
                            //          (turnDirA != 0)); 
                        }
                        VertID vID(b1.id, b1.vn);
                        //(*pointOrders)[b1].addOrderedPoints(&b1, &a1, reversed);
                    }
#endif
                }
            }
        }
        else
        {
            if (polyIsOrthogonal && connIsOrthogonal)
            {
                // All crossings in orthogonal connectors will be at a
                // vertex in the visibility graph, so we need not bother
                // doing normal line intersection.
                continue;
            }

            // No endpoint is shared between these two line segments,
            // so just calculate normal segment intersection.

            Point cPt;
            int   intersectResult = Avoid::segmentIntersectPoint(
                a1,
                a2,
                b1,
                b2,
                &(cPt.x),
                &(cPt.y)
            );

            if (intersectResult == Avoid::DO_INTERSECT)
            {
                if (!polyIsConn
                    && ((a1 == cPt) || (a2 == cPt) || (b1 == cPt) || (b2 == cPt)
                    ))
                {
                    // XXX: This shouldn't actually happen, because these
                    //      points should be added as bends to each line by
                    //      splitBranchingSegments().  Thus, lets ignore them.
                    COLA_ASSERT(a1 != cPt);
                    COLA_ASSERT(a2 != cPt);
                    COLA_ASSERT(b1 != cPt);
                    COLA_ASSERT(b2 != cPt);
                    continue;
                }
                // db_printf("crossing lines:\n");
                // db_printf("cPt: %g %g\n", cPt.x, cPt.y);
                crossingCount += 1;
                if (crossingPoints)
                {
                    crossingPoints->insert(cPt);
                }
            }
        }
    }
    // db_printf("crossingcount %d %d\n", crossingCount, crossingFlags);

    // Free shared path memory.
    delete[] c_path;
    delete[] p_path;
}

}  // namespace avoid
