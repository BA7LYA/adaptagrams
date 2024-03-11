///
/// @file ReferencingPolygon.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include <vector>

#include "libavoid/dllexport.hxx"
#include "libavoid/PolygonInterface.hxx"

namespace avoid {

class Point;
class Polygon;
class Router;

//! @brief  A Polygon which just references its points from other Polygons.
//!
//! This type of Polygon is used to accurately represent cluster boundaries
//! made up from the corner points of shapes.
//!
class AVOID_EXPORT ReferencingPolygon : public PolygonInterface
{
public:
    ReferencingPolygon();
    ReferencingPolygon(const Polygon& poly, const Router* router);

    void         clear(void);
    bool         empty(void) const;
    size_t       size(void) const;
    int          id(void) const;
    const Point& at(size_t index) const;

    int _id;

    std::vector<std::pair<const Polygon*, unsigned short>> psRef;
    std::vector<Point>                                     psPoints;
};

}  // namespace avoid
