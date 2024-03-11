///
/// @file ReferencingPolygon.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/ReferencingPolygon.hxx"

#include "libavoid/Polygon.hxx"

namespace avoid {

ReferencingPolygon::ReferencingPolygon(
    const Polygon& poly,
    const Router*  router
)
    : PolygonInterface()
    , _id(poly._id)
    , psRef(poly.size())
    , psPoints(poly.size())
{
    COLA_ASSERT(router != nullptr);
    for (size_t i = 0; i < poly.size(); ++i)
    {
        if (poly.ps[i].id == 0)
        {
            // Can't be referenced, so just make a copy of the point.
            psRef[i]
                = std::make_pair((Polygon*)nullptr, kUnassignedVertexNumber);
            psPoints[i] = poly.ps[i];
        }
        else
        {
            const Polygon* polyPtr = nullptr;
            for (ObstacleList::const_iterator sh = router->m_obstacles.begin();
                 sh != router->m_obstacles.end();
                 ++sh)
            {
                if ((*sh)->id() == poly.ps[i].id)
                {
                    const Polygon& poly = (*sh)->polygon();
                    polyPtr             = &poly;
                    break;
                }
            }
            COLA_ASSERT(polyPtr != nullptr);
            psRef[i] = std::make_pair(polyPtr, poly.ps[i].vn);
        }
    }
}

ReferencingPolygon::ReferencingPolygon()
    : PolygonInterface()
{
    clear();
}

void ReferencingPolygon::clear(void)
{
    psRef.clear();
    psPoints.clear();
}

bool ReferencingPolygon::empty(void) const
{
    return psRef.empty();
}

size_t ReferencingPolygon::size(void) const
{
    return psRef.size();
}

int ReferencingPolygon::id(void) const
{
    return _id;
}

const Point& ReferencingPolygon::at(size_t index) const
{
    COLA_ASSERT(index < size());

    if (psRef[index].first != nullptr)
    {
        const Polygon& poly       = *(psRef[index].first);
        unsigned short poly_index = psRef[index].second;
        COLA_ASSERT(poly_index < poly.size());

        return poly.ps[poly_index];
    }
    else
    {
        return psPoints[index];
    }
}

}  // namespace avoid
