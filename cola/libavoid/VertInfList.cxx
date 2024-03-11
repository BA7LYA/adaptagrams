///
/// @file VertInfList.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/VertInfList.hxx"

namespace avoid {

VertInfList::VertInfList()
    : _firstShapeVert(nullptr)
    , _firstConnVert(nullptr)
    , _lastShapeVert(nullptr)
    , _lastConnVert(nullptr)
    , _shapeVertices(0)
    , _connVertices(0)
{
}

void VertInfList::addVertex(VertInf* vert)
{
    checkVertInfListConditions();
    COLA_ASSERT(vert->lstPrev == nullptr);
    COLA_ASSERT(vert->lstNext == nullptr);

    if (vert->id.isConnPt())
    {
        // A Connector vertex
        if (_firstConnVert)
        {
            // Join with previous front
            vert->lstNext           = _firstConnVert;
            _firstConnVert->lstPrev = vert;

            // Make front
            _firstConnVert = vert;
        }
        else
        {
            // Make front and back
            _firstConnVert = vert;
            _lastConnVert  = vert;

            // Link to front of shapes list
            vert->lstNext = _firstShapeVert;
        }
        _connVertices++;
    }
    else  // if (vert->id.shape > 0)
    {
        // A Shape vertex
        if (_lastShapeVert)
        {
            // Join with previous back
            vert->lstPrev           = _lastShapeVert;
            _lastShapeVert->lstNext = vert;

            // Make back
            _lastShapeVert = vert;
        }
        else
        {
            // Make first and last
            _firstShapeVert = vert;
            _lastShapeVert  = vert;

            // Join with conns list
            if (_lastConnVert)
            {
                COLA_ASSERT(_lastConnVert->lstNext == nullptr);

                _lastConnVert->lstNext = vert;
            }
        }
        _shapeVertices++;
    }
    checkVertInfListConditions();
}

// Removes a vertex from the list and returns a pointer to the vertex
// following the removed one.
VertInf* VertInfList::removeVertex(VertInf* vert)
{
    if (vert == nullptr)
    {
        return nullptr;
    }
    // Conditions for correct data structure
    checkVertInfListConditions();

    VertInf* following = vert->lstNext;

    if (vert->id.isConnPt())
    {
        // A Connector vertex
        if (vert == _firstConnVert)
        {
            if (vert == _lastConnVert)
            {
                _firstConnVert = nullptr;
                _lastConnVert  = nullptr;
            }
            else
            {
                // Set new first
                _firstConnVert = _firstConnVert->lstNext;

                if (_firstConnVert)
                {
                    // Set previous
                    _firstConnVert->lstPrev = nullptr;
                }
            }
        }
        else if (vert == _lastConnVert)
        {
            // Set new last
            _lastConnVert = _lastConnVert->lstPrev;

            // Make last point to shapes list
            _lastConnVert->lstNext = _firstShapeVert;
        }
        else
        {
            vert->lstNext->lstPrev = vert->lstPrev;
            vert->lstPrev->lstNext = vert->lstNext;
        }
        _connVertices--;
    }
    else  // if (vert->id.shape > 0)
    {
        // A Shape vertex
        if (vert == _lastShapeVert)
        {
            // Set new last
            _lastShapeVert = _lastShapeVert->lstPrev;

            if (vert == _firstShapeVert)
            {
                _firstShapeVert = nullptr;
                if (_lastConnVert)
                {
                    _lastConnVert->lstNext = nullptr;
                }
            }

            if (_lastShapeVert)
            {
                _lastShapeVert->lstNext = nullptr;
            }
        }
        else if (vert == _firstShapeVert)
        {
            // Set new first
            _firstShapeVert = _firstShapeVert->lstNext;

            // Correct the last conn vertex
            if (_lastConnVert)
            {
                _lastConnVert->lstNext = _firstShapeVert;
            }

            if (_firstShapeVert)
            {
                _firstShapeVert->lstPrev = nullptr;
            }
        }
        else
        {
            vert->lstNext->lstPrev = vert->lstPrev;
            vert->lstPrev->lstNext = vert->lstNext;
        }
        _shapeVertices--;
    }
    vert->lstPrev = nullptr;
    vert->lstNext = nullptr;

    checkVertInfListConditions();

    return following;
}

VertInf* VertInfList::getVertexByID(const VertID& id)
{
    VertID searchID = id;
    if (searchID.vn == kUnassignedVertexNumber)
    {
        unsigned int topbit = ((unsigned int)1) << 31;
        if (searchID.objID & topbit)
        {
            searchID.objID = searchID.objID & ~topbit;
            searchID.vn    = VertID::src;
        }
        else
        {
            searchID.vn = VertID::tar;
        }
    }
    VertInf* last = end();
    for (VertInf* curr = connsBegin(); curr != last; curr = curr->lstNext)
    {
        if (curr->id == searchID)
        {
            return curr;
        }
    }
    return nullptr;
}

VertInf* VertInfList::getVertexByPos(const Point& p)
{
    VertInf* last = end();
    for (VertInf* curr = shapesBegin(); curr != last; curr = curr->lstNext)
    {
        if (curr->point == p)
        {
            return curr;
        }
    }
    return nullptr;
}

VertInf* VertInfList::shapesBegin(void)
{
    return _firstShapeVert;
}

VertInf* VertInfList::connsBegin(void)
{
    if (_firstConnVert)
    {
        return _firstConnVert;
    }
    // No connector vertices
    return _firstShapeVert;
}

VertInf* VertInfList::end(void)
{
    return nullptr;
}

unsigned int VertInfList::connsSize(void) const
{
    return _connVertices;
}

unsigned int VertInfList::shapesSize(void) const
{
    return _shapeVertices;
}

}  // namespace avoid
