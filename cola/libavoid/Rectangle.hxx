///
/// @file Rectangle.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/dllexport.hxx"
#include "libavoid/Polygon.hxx"

namespace avoid {

class Point;

//! @brief  A Rectangle, a simpler way to define the polygon for square or
//!         rectangular shapes.
//!
class AVOID_EXPORT Rectangle : public Polygon
{
public:
    //! @brief  Constructs a rectangular polygon given two opposing
    //!         corner points.
    //!
    //! @param[in]  topLeft      The first corner point of the rectangle.
    //! @param[in]  bottomRight  The opposing corner point of the rectangle.
    //!
    Rectangle(const Point& topLeft, const Point& bottomRight);

    //! @brief  Constructs a rectangular polygon given the centre, width
    //!         and height.
    //!
    //! @param[in]  centre  The centre of the rectangle, specified as
    //!                     a point.
    //! @param[in]  width   The width of the rectangle.
    //! @param[in]  height  The height of the rectangle.
    //!
    Rectangle(const Point& centre, const double width, const double height);
};

}  // namespace avoid
