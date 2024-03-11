///
/// @file Rectangle.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/Rectangle.hxx"

#include "libavoid/Point.hxx"
#include "libavoid/Polygon.hxx"

namespace avoid {

Rectangle::Rectangle(const Point& topLeft, const Point& bottomRight)
    : Polygon(4)
{
    double xMin = std::min(topLeft.x, bottomRight.x);
    double xMax = std::max(topLeft.x, bottomRight.x);
    double yMin = std::min(topLeft.y, bottomRight.y);
    double yMax = std::max(topLeft.y, bottomRight.y);

    ps[0] = Point(xMax, yMin);
    ps[1] = Point(xMax, yMax);
    ps[2] = Point(xMin, yMax);
    ps[3] = Point(xMin, yMin);
}

Rectangle::Rectangle(
    const Point& centre,
    const double width,
    const double height
)
    : Polygon(4)
{
    double halfWidth  = width / 2.0;
    double halfHeight = height / 2.0;
    double xMin       = centre.x - halfWidth;
    double xMax       = centre.x + halfWidth;
    double yMin       = centre.y - halfHeight;
    double yMax       = centre.y + halfHeight;

    ps[0] = Point(xMax, yMin);
    ps[1] = Point(xMax, yMax);
    ps[2] = Point(xMin, yMax);
    ps[3] = Point(xMin, yMin);
}

}  // namespace avoid
