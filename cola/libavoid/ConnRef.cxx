///
/// @file ConnRef.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/ConnRef.hxx"

namespace avoid {

ConnRef::ConnRef(Router* router, const unsigned int id)
    : m_router(router)
    , m_type(router->validConnType())
    , m_reroute_flag_ptr(nullptr)
    , m_needs_reroute_flag(true)
    , m_false_path(false)
    , m_needs_repaint(false)
    , m_active(false)
    , m_hate_crossings(false)
    , m_has_fixed_route(false)
    , m_route_dist(0)
    , m_src_vert(nullptr)
    , m_dst_vert(nullptr)
    , m_start_vert(nullptr)
    , m_callback_func(nullptr)
    , m_connector(nullptr)
    , m_src_connend(nullptr)
    , m_dst_connend(nullptr)
{
    COLA_ASSERT(m_router != nullptr);
    m_id = m_router->assignId(id);

    // TODO: Store endpoints and details.
    m_route.clear();

    m_reroute_flag_ptr = m_router->m_conn_reroute_flags.addConn(this);
}

ConnRef::ConnRef(
    Router*            router,
    const ConnEnd&     src,
    const ConnEnd&     dst,
    const unsigned int id
)
    : m_router(router)
    , m_type(router->validConnType())
    , m_reroute_flag_ptr(nullptr)
    , m_needs_reroute_flag(true)
    , m_false_path(false)
    , m_needs_repaint(false)
    , m_active(false)
    , m_hate_crossings(false)
    , m_has_fixed_route(false)
    , m_route_dist(0)
    , m_src_vert(nullptr)
    , m_dst_vert(nullptr)
    , m_callback_func(nullptr)
    , m_connector(nullptr)
    , m_src_connend(nullptr)
    , m_dst_connend(nullptr)
{
    COLA_ASSERT(m_router != nullptr);
    m_id = m_router->assignId(id);
    m_route.clear();

    // Set endpoint values.
    setEndpoints(src, dst);

    m_reroute_flag_ptr = m_router->m_conn_reroute_flags.addConn(this);
}

ConnRef::~ConnRef()
{
    COLA_ASSERT(m_router);

    if (m_router->m_currently_calling_destructors == false)
    {
        err_printf("ERROR: ConnRef::~ConnRef() shouldn't be called directly.\n"
        );
        err_printf(
            "       It is owned by the router.  Call Router::deleteConnector() "
            "instead.\n"
        );
        abort();
    }

    m_router->m_conn_reroute_flags.removeConn(this);

    m_router->removeObjectFromQueuedActions(this);

    freeRoutes();

    if (m_src_vert)
    {
        m_src_vert->removeFromGraph();
        m_router->vertices.removeVertex(m_src_vert);
        delete m_src_vert;
        m_src_vert = nullptr;
    }
    if (m_src_connend)
    {
        m_src_connend->disconnect();
        m_src_connend->freeActivePin();
        delete m_src_connend;
        m_src_connend = nullptr;
    }

    if (m_dst_vert)
    {
        m_dst_vert->removeFromGraph();
        m_router->vertices.removeVertex(m_dst_vert);
        delete m_dst_vert;
        m_dst_vert = nullptr;
    }
    if (m_dst_connend)
    {
        m_dst_connend->disconnect();
        m_dst_connend->freeActivePin();
        delete m_dst_connend;
        m_dst_connend = nullptr;
    }

    // Clear checkpoint vertices.
    for (size_t i = 0; i < m_checkpoint_vertices.size(); ++i)
    {
        m_checkpoint_vertices[i]->removeFromGraph(true);
        m_router->vertices.removeVertex(m_checkpoint_vertices[i]);
        delete m_checkpoint_vertices[i];
    }
    m_checkpoint_vertices.clear();

    if (m_active)
    {
        makeInactive();
    }
}

ConnType ConnRef::routingType(void) const
{
    return m_type;
}

void ConnRef::setRoutingType(ConnType type)
{
    type = m_router->validConnType(type);
    if (m_type != type)
    {
        m_type = type;

        makePathInvalid();

        m_router->modifyConnector(this);
    }
}

std::vector<Checkpoint> ConnRef::routingCheckpoints(void) const
{
    return m_checkpoints;
}

void ConnRef::setRoutingCheckpoints(const std::vector<Checkpoint>& checkpoints)
{
    m_checkpoints = checkpoints;

    // Clear previous checkpoint vertices.
    for (size_t i = 0; i < m_checkpoint_vertices.size(); ++i)
    {
        m_checkpoint_vertices[i]->removeFromGraph(true);
        m_router->vertices.removeVertex(m_checkpoint_vertices[i]);
        delete m_checkpoint_vertices[i];
    }
    m_checkpoint_vertices.clear();

    for (size_t i = 0; i < m_checkpoints.size(); ++i)
    {
        VertID ptID(
            m_id,
            2 + i,
            VertID::PROP_ConnPoint | VertID::PROP_ConnCheckpoint
        );
        VertInf* vertex = new VertInf(m_router, ptID, m_checkpoints[i].point);
        vertex->visDirections = ConnDirAll;

        m_checkpoint_vertices.push_back(vertex);
    }
    if (m_router->m_allows_polyline_routing)
    {
        for (size_t i = 0; i < m_checkpoints.size(); ++i)
        {
            vertexVisibility(m_checkpoint_vertices[i], nullptr, true, true);
        }
    }
}

void ConnRef::common_updateEndPoint(const unsigned int type, ConnEnd connEnd)
{
    const Point& point = connEnd.position();
    // db_printf("common_updateEndPoint(%d,(pid=%d,vn=%d,(%f,%f)))\n",
    //       type,point.id,point.vn,point.x,point.y);
    COLA_ASSERT(
        (type == (unsigned int)VertID::src)
        || (type == (unsigned int)VertID::tar)
    );

    // The connEnd is a copy of a ConnEnd that will get disconnected,
    // so don't leave it looking like it is still connected.
    connEnd.m_conn_ref = nullptr;

    if (!m_active)
    {
        makeActive();
    }

    VertInf* altered = nullptr;

    VertIDProps properties = VertID::PROP_ConnPoint;
    if (connEnd.isPinConnection())
    {
        properties |= VertID::PROP_DummyPinHelper;
    }
    VertID ptID(m_id, type, properties);
    if (type == (unsigned int)VertID::src)
    {
        if (m_src_vert)
        {
            m_src_vert->Reset(ptID, point);
        }
        else
        {
            m_src_vert = new VertInf(m_router, ptID, point);
        }
        m_src_vert->visDirections = connEnd.directions();

        if (m_src_connend)
        {
            m_src_connend->disconnect();
            m_src_connend->freeActivePin();
            delete m_src_connend;
            m_src_connend = nullptr;
        }
        if (connEnd.isPinConnection())
        {
            m_src_connend = new ConnEnd(connEnd);
            m_src_connend->connect(this);
            // Don't need this to have visibility since we won't
            // be connecting to it.
            m_src_vert->visDirections = ConnDirNone;
        }

        altered = m_src_vert;
    }
    else  // if (type == (unsigned int) VertID::tar)
    {
        if (m_dst_vert)
        {
            m_dst_vert->Reset(ptID, point);
        }
        else
        {
            m_dst_vert = new VertInf(m_router, ptID, point);
        }
        m_dst_vert->visDirections = connEnd.directions();

        if (m_dst_connend)
        {
            m_dst_connend->disconnect();
            m_dst_connend->freeActivePin();
            delete m_dst_connend;
            m_dst_connend = nullptr;
        }
        if (connEnd.isPinConnection())
        {
            m_dst_connend = new ConnEnd(connEnd);
            m_dst_connend->connect(this);
            // Don't need this to have visibility since we won't
            // be connecting to it.
            m_dst_vert->visDirections = ConnDirNone;
        }

        altered = m_dst_vert;
    }

    // XXX: Seems to be faster to just remove the edges and recreate
    bool isConn = true;
    altered->removeFromGraph(isConn);

    makePathInvalid();
    m_router->setStaticGraphInvalidated(true);
}

void ConnRef::setEndpoints(const ConnEnd& srcPoint, const ConnEnd& dstPoint)
{
    m_router->modifyConnector(this, VertID::src, srcPoint);
    m_router->modifyConnector(this, VertID::tar, dstPoint);
}

void ConnRef::setEndpoint(const unsigned int type, const ConnEnd& connEnd)
{
    m_router->modifyConnector(this, type, connEnd);
}

void ConnRef::setSourceEndpoint(const ConnEnd& srcPoint)
{
    m_router->modifyConnector(this, VertID::src, srcPoint);
}

void ConnRef::setDestEndpoint(const ConnEnd& dstPoint)
{
    m_router->modifyConnector(this, VertID::tar, dstPoint);
}

// Given the start or end vertex of a connector, returns the ConnEnd that
// can be used to reproduce that endpoint.  This is used for hyperedge routing.
//
bool ConnRef::getConnEndForEndpointVertex(VertInf* vertex, ConnEnd& connEnd)
    const
{
    if (vertex == nullptr)
    {
        err_printf(
            "Warning: In ConnRef::getConnEndForEndpointVertex():\n"
            "         ConnEnd for connector %d is uninitialised.  It may have "
            "been\n"
            "         set but Router::processTrancaction has not yet been "
            "called.\n",
            (int)id()
        );
        return false;
    }

    if (vertex == m_src_vert)
    {
        if (m_src_connend)
        {
            connEnd = *m_src_connend;
        }
        else
        {
            connEnd = ConnEnd(
                Point(m_src_vert->point.x, m_src_vert->point.y),
                m_src_vert->visDirections
            );
        }
        return true;
    }
    else if (vertex == m_dst_vert)
    {
        if (m_dst_connend)
        {
            connEnd = *m_dst_connend;
        }
        else
        {
            connEnd = ConnEnd(
                Point(m_dst_vert->point.x, m_dst_vert->point.y),
                m_dst_vert->visDirections
            );
        }
        return true;
    }
    return false;
}

void ConnRef::updateEndPoint(const unsigned int type, const ConnEnd& connEnd)
{
    common_updateEndPoint(type, connEnd);

    if (m_has_fixed_route)
    {
        // Don't need to continue and compute visibility if route is fixed.
        return;
    }

    if (m_router->m_allows_polyline_routing)
    {
        bool knownNew    = true;
        bool genContains = true;
        if (type == (unsigned int)VertID::src)
        {
            bool dummySrc = m_src_connend && m_src_connend->isPinConnection();
            if (!dummySrc)
            {
                // Only generate visibility if not attached to a pin.
                vertexVisibility(m_src_vert, m_dst_vert, knownNew, genContains);
            }
        }
        else
        {
            bool dummyDst = m_dst_connend && m_dst_connend->isPinConnection();
            if (!dummyDst)
            {
                // Only generate visibility if not attached to a pin.
                vertexVisibility(m_dst_vert, m_src_vert, knownNew, genContains);
            }
        }
    }
}

void ConnRef::outputCode(FILE* fp) const
{
    fprintf(fp, "    // connRef%u\n", id());
    fprintf(fp, "    connRef = new ConnRef(router, %u);\n", id());
    if (m_src_connend)
    {
        m_src_connend->outputCode(fp, "src");
        fprintf(fp, "    connRef->setSourceEndpoint(srcPt);\n");
    }
    else if (src())
    {
        fprintf(
            fp,
            "    srcPt = ConnEnd(Point(%" PREC "g, %" PREC "g), %u);\n",
            src()->point.x,
            src()->point.y,
            src()->visDirections
        );
        fprintf(fp, "    connRef->setSourceEndpoint(srcPt);\n");
    }
    if (m_dst_connend)
    {
        m_dst_connend->outputCode(fp, "dst");
        fprintf(fp, "    connRef->setDestEndpoint(dstPt);\n");
    }
    else if (dst())
    {
        fprintf(
            fp,
            "    dstPt = ConnEnd(Point(%" PREC "g, %" PREC "g), %u);\n",
            dst()->point.x,
            dst()->point.y,
            dst()->visDirections
        );
        fprintf(fp, "    connRef->setDestEndpoint(dstPt);\n");
    }
    fprintf(fp, "    connRef->setRoutingType((ConnType)%u);\n", routingType());

    if (m_has_fixed_route)
    {
        PolyLine currRoute = route();
        fprintf(fp, "    newRoute._id = %u;\n", id());
        fprintf(fp, "    newRoute.ps.resize(%d);\n", (int)currRoute.size());
        for (size_t i = 0; i < currRoute.size(); ++i)
        {
            fprintf(
                fp,
                "    newRoute.ps[%d] = Point(%" PREC "g, %" PREC "g);\n",
                (int)i,
                currRoute.ps[i].x,
                currRoute.ps[i].y
            );
            fprintf(
                fp,
                "    newRoute.ps[%d].id = %u;\n",
                (int)i,
                currRoute.ps[i].id
            );
            fprintf(
                fp,
                "    newRoute.ps[%d].vn = %u;\n",
                (int)i,
                currRoute.ps[i].vn
            );
        }
        fprintf(fp, "    connRef->setFixedRoute(newRoute);\n");
    }

    if (!m_checkpoints.empty())
    {
        fprintf(
            fp,
            "    std::vector<Checkpoint> checkpoints%u(%d);\n",
            id(),
            (int)m_checkpoints.size()
        );
        for (size_t cInd = 0; cInd < m_checkpoints.size(); ++cInd)
        {
            fprintf(
                fp,
                "    checkpoints%u[%d] = Checkpoint(Point("
                "%" PREC "g, %" PREC
                "g), (ConnDirFlags) %d, "
                "(ConnDirFlags) %d);\n",
                id(),
                (int)cInd,
                m_checkpoints[cInd].point.x,
                m_checkpoints[cInd].point.y,
                m_checkpoints[cInd].arrivalDirections,
                m_checkpoints[cInd].departureDirections
            );
        }
        fprintf(
            fp,
            "    connRef->setRoutingCheckpoints(checkpoints%u);\n",
            id()
        );
    }
    fprintf(fp, "\n");
}

std::pair<Obstacle*, Obstacle*> ConnRef::endpointAnchors(void) const
{
    std::pair<Obstacle*, Obstacle*> anchors;
    anchors.first  = nullptr;
    anchors.second = nullptr;

    if (m_src_connend)
    {
        anchors.first = m_src_connend->m_anchor_obj;
    }
    if (m_dst_connend)
    {
        anchors.second = m_dst_connend->m_anchor_obj;
    }
    return anchors;
}

std::pair<ConnEnd, ConnEnd> ConnRef::endpointConnEnds(void) const
{
    std::pair<ConnEnd, ConnEnd> endpoints;
    getConnEndForEndpointVertex(m_src_vert, endpoints.first);
    getConnEndForEndpointVertex(m_dst_vert, endpoints.second);
    return endpoints;
}

bool ConnRef::setEndpoint(
    const unsigned int type,
    const VertID&      pointID,
    Point*             pointSuggestion
)
{
    VertInf* vInf = m_router->vertices.getVertexByID(pointID);
    if (vInf == nullptr)
    {
        return false;
    }
    Point& point = vInf->point;
    if (pointSuggestion)
    {
        if (euclideanDist(point, *pointSuggestion) > 0.5)
        {
            return false;
        }
    }

    common_updateEndPoint(type, point);

    // Give this visibility just to the point it is over.
    EdgeInf* edge
        = new EdgeInf((type == VertID::src) ? m_src_vert : m_dst_vert, vInf);
    // XXX: We should be able to set this to zero, but can't due to
    //      assumptions elsewhere in the code.
    edge->setDist(0.001);

    m_router->processTransaction();
    return true;
}

void ConnRef::makeActive(void)
{
    COLA_ASSERT(!m_active);

    // Add to connRefs list.
    m_connrefs_pos
        = m_router->connRefs.insert(m_router->connRefs.begin(), this);
    m_active = true;
}

void ConnRef::freeActivePins(void)
{
    if (m_src_connend)
    {
        m_src_connend->freeActivePin();
    }
    if (m_dst_connend)
    {
        m_dst_connend->freeActivePin();
    }
}

void ConnRef::makeInactive(void)
{
    COLA_ASSERT(m_active);

    // Remove from connRefs list.
    m_router->connRefs.erase(m_connrefs_pos);
    m_active = false;
}

void ConnRef::freeRoutes(void)
{
    m_route.clear();
    m_display_route.clear();
}

const PolyLine& ConnRef::route(void) const
{
    return m_route;
}

PolyLine& ConnRef::routeRef(void)
{
    return m_route;
}

void ConnRef::set_route(const PolyLine& route)
{
    if (&m_display_route == &route)
    {
        db_printf("Error:\tTrying to update libavoid route with itself.\n");
        return;
    }
    m_display_route.ps = route.ps;

    //_display_route.clear();
}

void ConnRef::setFixedExistingRoute(void)
{
    COLA_ASSERT(m_route.size() >= 2);
    m_has_fixed_route = true;
    m_router->registerSettingsChange();
}

void ConnRef::setFixedRoute(const PolyLine& route)
{
    if (route.size() >= 2)
    {
        // Set endpoints based on the fixed route incase the
        // fixed route is later cleared.
        setEndpoints(route.ps[0], route.ps[route.size() - 1]);
    }
    m_has_fixed_route = true;
    m_route           = route;
    m_display_route   = m_route.simplify();
    m_router->registerSettingsChange();
}

bool ConnRef::hasFixedRoute(void) const
{
    return m_has_fixed_route;
}

void ConnRef::clearFixedRoute(void)
{
    m_has_fixed_route = false;
    makePathInvalid();
    m_router->registerSettingsChange();
}

Polygon& ConnRef::displayRoute(void)
{
    if (m_display_route.empty())
    {
        // No displayRoute is set.  Simplify the current route to get it.
        m_display_route = m_route.simplify();
    }
    return m_display_route;
}

void ConnRef::calcRouteDist(void)
{
    double (*dist)(const Point& a, const Point& b)
        = (m_type == ConnType_PolyLine) ? euclideanDist : manhattanDist;

    m_route_dist = 0;
    for (size_t i = 1; i < m_route.size(); ++i)
    {
        m_route_dist += dist(m_route.at(i), m_route.at(i - 1));
    }
}

bool ConnRef::needsRepaint(void) const
{
    return m_needs_repaint;
}

unsigned int ConnRef::id(void) const
{
    return m_id;
}

std::pair<JunctionRef*, ConnRef*> ConnRef::splitAtSegment(const size_t segmentN)
{
    ConnRef*     newConn     = nullptr;
    JunctionRef* newJunction = nullptr;

    if (m_display_route.size() > segmentN)
    {
        // Position the junction at the midpoint of the desired segment.
        Point junctionPos = midpoint(
            m_display_route.at(segmentN - 1),
            m_display_route.at(segmentN)
        );

        // Create the new junction.
        newJunction = new JunctionRef(router(), junctionPos);
        router()->addJunction(newJunction);
        newJunction->preferOrthogonalDimension(
            (m_display_route.at(segmentN - 1).x
             == m_display_route.at(segmentN).x)
                ? YDIM
                : XDIM
        );

        // Create a new connection routing from the junction to the original
        // connector's endpoint.
        ConnEnd newConnSrc = ConnEnd(newJunction);
        ConnEnd newConnDst = *m_dst_connend;
        newConn            = new ConnRef(router(), newConnSrc, newConnDst);

        // Reroute the endpoint of the original connector to attach to the
        // new junction.
        ConnEnd oldConnDst = ConnEnd(newJunction);
        this->setDestEndpoint(oldConnDst);
    }

    return std::make_pair(newJunction, newConn);
}

VertInf* ConnRef::src(void) const
{
    return m_src_vert;
}

VertInf* ConnRef::dst(void) const
{
    return m_dst_vert;
}

VertInf* ConnRef::start(void)
{
    return m_start_vert;
}

bool ConnRef::isInitialised(void) const
{
    return m_active;
}

void ConnRef::unInitialise(void)
{
    m_router->vertices.removeVertex(m_src_vert);
    m_router->vertices.removeVertex(m_dst_vert);
    makeInactive();
}

void ConnRef::removeFromGraph(void)
{
    if (m_src_vert)
    {
        m_src_vert->removeFromGraph();
    }

    if (m_dst_vert)
    {
        m_dst_vert->removeFromGraph();
    }
}

void ConnRef::setCallback(void (*cb)(void*), void* ptr)
{
    m_callback_func = cb;
    m_connector     = ptr;
}

void ConnRef::performCallback(void)
{
    if (m_callback_func)
    {
        m_callback_func(m_connector);
    }
}

void ConnRef::makePathInvalid(void)
{
    m_needs_reroute_flag = true;
}

Router* ConnRef::router(void) const
{
    return m_router;
}

std::pair<bool, bool> ConnRef::assignConnectionPinVisibility(const bool connect)
{
    // XXX This is kind of a hack for connection pins.  Probably we want to
    //     not use m_src_vert and m_dst_vert.  For the moment we will clear
    //     their visibility and give them visibility to the pins.
    bool dummySrc = m_src_connend && m_src_connend->isPinConnection();
    if (dummySrc)
    {
        m_src_vert->removeFromGraph();
        if (connect)
        {
            m_src_connend->assignPinVisibilityTo(m_src_vert, m_dst_vert);
        }
    }
    bool dummyDst = m_dst_connend && m_dst_connend->isPinConnection();
    if (dummyDst)
    {
        m_dst_vert->removeFromGraph();
        if (connect)
        {
            m_dst_connend->assignPinVisibilityTo(m_dst_vert, m_src_vert);
        }
    }

    return std::make_pair(dummySrc, dummyDst);
}

bool ConnRef::generatePath(void)
{
    // XXX Currently rubber-band routing only works when dragging the
    //     destination point of a connector, but not the source.  The code
    //     needs to be reworked to work in both directions.

    if (!m_false_path && !m_needs_reroute_flag)
    {
        // This connector is up to date.
        return false;
    }

    if (!m_dst_vert || !m_src_vert)
    {
        // Connector is not fully initialised.
        return false;
    }

    // COLA_ASSERT(_srcVert->point != _dstVert->point);

    m_false_path         = false;
    m_needs_reroute_flag = false;

    m_start_vert = m_src_vert;

    // Some connectors may attach to connection pins, which means they route
    // to the closest of multiple pins on a shape.  How we handle this is to
    // add a dummy vertex as the source or target vertex.  This is then given
    // visibility to each of the possible pins and tiny distance.  Here we
    // assign this visibility by adding edges to the visibility graph that we
    // later remove.
    std::pair<bool, bool> isDummyAtEnd = assignConnectionPinVisibility(true);

    if (m_router->RubberBandRouting && route().size() > 0)
    {
        if (isDummyAtEnd.first)
        {
            // ShapeConnectionPin *activePin = m_src_connend->active
            Point firstPoint        = m_src_vert->point;
            firstPoint.id           = m_src_vert->id.objID;
            firstPoint.vn           = m_src_vert->id.vn;
            PolyLine& existingRoute = routeRef();
            existingRoute.ps.insert(existingRoute.ps.begin(), 1, firstPoint);
        }
    }

    std::vector<Point>    path;
    std::vector<VertInf*> vertices;
    if (m_checkpoints.empty())
    {
        generateStandardPath(path, vertices);
    }
    else
    {
        generateCheckpointsPath(path, vertices);
    }

    COLA_ASSERT(vertices.size() >= 2);
    COLA_ASSERT(vertices[0] == src());
    COLA_ASSERT(vertices[vertices.size() - 1] == dst());
    COLA_ASSERT(m_reroute_flag_ptr != nullptr);

    for (size_t i = 1; i < vertices.size(); ++i)
    {
        if (m_router->InvisibilityGrph && (m_type == ConnType_PolyLine))
        {
            // TODO: Again, we could know this edge without searching.
            EdgeInf* edge = EdgeInf::existingEdge(vertices[i - 1], vertices[i]);
            if (edge)
            {
                edge->addConn(m_reroute_flag_ptr);
            }
        }
        else
        {
            m_false_path = true;
        }

        VertInf* vertex = vertices[i];
        if (vertex->pathNext && (vertex->pathNext->point == vertex->point))
        {
            if (!(vertex->pathNext->id.isConnPt()) && !(vertex->id.isConnPt()))
            {
                // Check for consecutive points on opposite
                // corners of two touching shapes.
                COLA_ASSERT(abs(vertex->pathNext->id.vn - vertex->id.vn) != 2);
            }
        }
    }

    // Get rid of dummy ShapeConnectionPin bridging points at beginning
    // and end of path.
    std::vector<Point>           clippedPath;
    std::vector<Point>::iterator pathBegin = path.begin();
    std::vector<Point>::iterator pathEnd   = path.end();
    if (path.size() > 2 && isDummyAtEnd.first)
    {
        ++pathBegin;
        m_src_connend->usePinVertex(vertices[1]);
    }
    if (path.size() > 2 && isDummyAtEnd.second)
    {
        --pathEnd;
        m_dst_connend->usePinVertex(vertices[vertices.size() - 2]);
    }
    clippedPath.insert(clippedPath.end(), pathBegin, pathEnd);

    // Clear visibility edges added for connection pins dummy vertices.
    assignConnectionPinVisibility(false);

    freeRoutes();
    PolyLine& output_route = m_route;
    output_route.ps        = clippedPath;

#ifdef PATHDEBUG
    db_printf("Output route:\n");
    for (size_t i = 0; i < output_route.ps.size(); ++i)
    {
        db_printf(
            "[%d,%d] %g, %g   ",
            output_route.ps[i].id,
            output_route.ps[i].vn,
            output_route.ps[i].x,
            output_route.ps[i].y
        );
    }
    db_printf("\n\n");
#endif

#ifdef DEBUGHANDLER
    if (m_router->debugHandler())
    {
        m_router->debugHandler()->updateConnectorRoute(this, -1, -1);
    }
#endif

    return true;
}

void ConnRef::generateCheckpointsPath(
    std::vector<Point>&    path,
    std::vector<VertInf*>& vertices
)
{
    std::vector<VertInf*> checkpoints = m_checkpoint_vertices;
    checkpoints.insert(checkpoints.begin(), src());
    checkpoints.push_back(dst());

    path.clear();
    vertices.clear();
    path.push_back(src()->point);
    vertices.push_back(src());

    size_t lastSuccessfulIndex = 0;
    for (size_t i = 1; i < checkpoints.size(); ++i)
    {
        VertInf* start = checkpoints[lastSuccessfulIndex];
        VertInf* end   = checkpoints[i];

        // Handle checkpoint directions by disabling some visibility edges.
        if (lastSuccessfulIndex > 0)
        {
            Checkpoint& srcCP = m_checkpoints[lastSuccessfulIndex - 1];
            if (srcCP.departureDirections != ConnDirAll)
            {
                start->setVisibleDirections(srcCP.departureDirections);
            }
        }
        if ((i + 1) < checkpoints.size())
        {
            Checkpoint& dstCP = m_checkpoints[i - 1];
            if (dstCP.arrivalDirections != ConnDirAll)
            {
                end->setVisibleDirections(dstCP.arrivalDirections);
            }
        }

        AStarPath aStar;
        // Route the connector
        aStar.search(this, start, end, nullptr);

        // Restore changes made for checkpoint visibility directions.
        if (lastSuccessfulIndex > 0)
        {
            start->setVisibleDirections(ConnDirAll);
        }
        if ((i + 1) < checkpoints.size())
        {
            end->setVisibleDirections(ConnDirAll);
        }

        // Process the path.
        int pathlen = end->pathLeadsBackTo(start);
        if (pathlen >= 2)
        {
            size_t prev_path_size = path.size();
            path.resize(prev_path_size + (pathlen - 1));
            vertices.resize(prev_path_size + (pathlen - 1));
            VertInf* vertInf = end;
            for (size_t index = path.size() - 1; index >= prev_path_size;
                 --index)
            {
                path[index] = vertInf->point;
                if (vertInf->id.isConnPt())
                {
                    path[index].id = m_id;
                    path[index].vn = kUnassignedVertexNumber;
                }
                else
                {
                    path[index].id = vertInf->id.objID;
                    path[index].vn = vertInf->id.vn;
                }
                vertices[index] = vertInf;
                vertInf         = vertInf->pathNext;
            }
            lastSuccessfulIndex = i;
        }
        else if (i + 1 == checkpoints.size())
        {
            // There is no valid path.
            db_printf("Warning: Path not found...\n");
            m_needs_reroute_flag = true;

            path.push_back(dst()->point);
            vertices.push_back(dst());

            COLA_ASSERT(path.size() >= 2);
        }
        else
        {
            err_printf(
                "Warning: skipping checkpoint for connector "
                "%d at (%g, %g).\n",
                (int)id(),
                checkpoints[i]->point.x,
                checkpoints[i]->point.y
            );
            fflush(stderr);
        }
    }
    // Use topbit to differentiate between start and end point of connector.
    // They need unique IDs for nudging.
    unsigned int topbit      = ((unsigned int)1) << 31;
    path[path.size() - 1].id = m_id | topbit;
    path[path.size() - 1].vn = kUnassignedVertexNumber;
}

void ConnRef::generateStandardPath(
    std::vector<Point>&    path,
    std::vector<VertInf*>& vertices
)
{
    VertInf*        tar               = m_dst_vert;
    size_t          existingPathStart = 0;
    const PolyLine& currRoute         = route();
    if (m_router->RubberBandRouting)
    {
        COLA_ASSERT(m_router->IgnoreRegions == true);

#ifdef PATHDEBUG
        db_printf("\n");
        src()->id.db_print();
        db_printf(": %g, %g\n", src()->point.x, src()->point.y);
        tar->id.db_print();
        db_printf(": %g, %g\n", tar->point.x, tar->point.y);
        for (size_t i = 0; i < currRoute.ps.size(); ++i)
        {
            db_printf("%g, %g  ", currRoute.ps[i].x, currRoute.ps[i].y);
        }
        db_printf("\n");
#endif
        if (currRoute.size() > 2)
        {
            if (m_src_vert->point == currRoute.ps[0])
            {
                existingPathStart = currRoute.size() - 2;
                COLA_ASSERT(existingPathStart != 0);
                const Point& pnt = currRoute.at(existingPathStart);
                VertID       vID(pnt.id, pnt.vn);

                m_start_vert = m_router->vertices.getVertexByID(vID);
                COLA_ASSERT(m_start_vert);
            }
        }
    }
    // db_printf("GO\n");
    // db_printf("src: %X strt: %X dst: %X\n", (int) m_src_vert, (int)
    // m_start_vert, (int) m_dst_vert);
    unsigned int pathlen = 0;
    while (pathlen == 0)
    {
        AStarPath aStar;
        aStar.search(this, src(), dst(), start());
        pathlen = dst()->pathLeadsBackTo(src());
        if (pathlen < 2)
        {
            if (existingPathStart == 0)
            {
                break;
            }
#ifdef PATHDEBUG
            db_printf("BACK\n");
#endif
            existingPathStart--;
            const Point& pnt = currRoute.at(existingPathStart);
            VertIDProps  props
                = (existingPathStart > 0) ? 0 : VertID::PROP_ConnPoint;
            VertID vID(pnt.id, pnt.vn, props);

            m_start_vert = m_router->vertices.getVertexByID(vID);
            COLA_ASSERT(m_start_vert);
        }
        else if (m_router->RubberBandRouting)
        {
            // found.
            bool unwind = false;

#ifdef PATHDEBUG
            db_printf("\n\n\nSTART:\n\n");
#endif
            VertInf* prior = nullptr;
            for (VertInf* curr = tar; curr != m_start_vert->pathNext;
                 curr          = curr->pathNext)
            {
                if (!validateBendPoint(curr->pathNext, curr, prior))
                {
                    unwind = true;
                    break;
                }
                prior = curr;
            }
            if (unwind)
            {
#ifdef PATHDEBUG
                db_printf("BACK II\n");
#endif
                if (existingPathStart == 0)
                {
                    break;
                }
                existingPathStart--;
                const Point& pnt = currRoute.at(existingPathStart);
                VertIDProps  props
                    = (existingPathStart > 0) ? 0 : VertID::PROP_ConnPoint;
                VertID vID(pnt.id, pnt.vn, props);

                m_start_vert = m_router->vertices.getVertexByID(vID);
                COLA_ASSERT(m_start_vert);

                pathlen = 0;
            }
        }
    }

    if (pathlen < 2)
    {
        // There is no valid path.
        db_printf("Warning: Path not found...\n");
        m_needs_reroute_flag = true;
        pathlen              = 2;
        tar->pathNext        = m_src_vert;
        if ((m_type == ConnType_PolyLine) && m_router->InvisibilityGrph)
        {
            // TODO:  Could we know this edge already?
            // EdgeInf *edge = EdgeInf::existingEdge(m_src_vert, tar);
            // COLA_ASSERT(edge != nullptr);
            // edge->addCycleBlocker();
        }
    }
    path.resize(pathlen);
    vertices.resize(pathlen);

    unsigned int j = pathlen - 1;
    for (VertInf* i = tar; i != m_src_vert; i = i->pathNext)
    {
        path[j]     = i->point;
        vertices[j] = i;
        path[j].id  = i->id.objID;
        path[j].vn  = i->id.vn;

        j--;
    }
    vertices[0] = m_src_vert;
    path[0]     = m_src_vert->point;
    path[0].id  = m_src_vert->id.objID;
    path[0].vn  = m_src_vert->id.vn;
}

void ConnRef::setHateCrossings(bool value)
{
    m_hate_crossings = value;
}

bool ConnRef::doesHateCrossings(void) const
{
    return m_hate_crossings;
}

std::vector<Point> ConnRef::possibleDstPinPoints(void) const
{
    std::vector<Point> points;
    if (m_dst_connend)
    {
        points = m_dst_connend->possiblePinPoints();
    }
    return points;
}

}  // namespace avoid
