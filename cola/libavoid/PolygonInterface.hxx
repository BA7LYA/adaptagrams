///
/// @file PolygonInterface.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/dllexport.hxx"

namespace avoid {

class Box;
class Point;
class Polygon;

//! @brief  A common interface used by the Polygon classes.
//!
class AVOID_EXPORT PolygonInterface
{
public:
    //! @brief  Constructor.
    PolygonInterface() {}

    //! @brief  Destructor.
    virtual ~PolygonInterface() {}

    //! @brief  Resets this to the empty polygon.
    virtual void clear(void) = 0;

    //! @brief  Returns true if this polygon is empty.
    virtual bool empty(void) const = 0;

    //! @brief  Returns the number of points in this polygon.
    virtual size_t size(void) const = 0;

    //! @brief  Returns the ID value associated with this polygon.
    virtual int id(void) const = 0;

    //! @brief  Returns a specific point in the polygon.
    //! @param[in]  index  The array index of the point to be returned.
    virtual const Point& at(size_t index) const = 0;

    //! @brief  Returns the bounding rectangle for this polygon.
    //!
    //! @return A new Rectangle representing the bounding box.
    Polygon boundingRectPolygon(void) const;

    //! @brief  Returns the bounding rectangle that contains this polygon
    //!         with optionally some buffer space around it for routing.
    //!
    //! If a buffer distance of zero is given, then this method returns
    //! the bounding rectangle for the shape's polygon.
    //!
    //! @param     offset  Extra distance to pad each side of the rect.
    //! @return    The bounding box for the polygon.
    Box offsetBoundingBox(double offset) const;

    Polygon offsetPolygon(double offset) const;
};

}  // namespace avoid
