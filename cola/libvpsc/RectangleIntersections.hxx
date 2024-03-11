///
/// @file RectangleIntersections.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace vpsc {

/* records the positions and sides through which a particular line intersects
 * with a rectangle
 */
struct RectangleIntersections
{
    bool   intersects, top, bottom, left, right;
    double topX, topY;
    double bottomX, bottomY;
    double leftX, leftY;
    double rightX, rightY;

    RectangleIntersections()
        : intersects(false)
        , top(false)
        , bottom(false)
        , left(false)
        , right(false)
        , topX(0)
        , topY(0)
        , bottomX(0)
        , bottomY(0)
        , leftX(0)
        , leftY(0)
        , rightX(0)
        , rightY(0)
    {
    }

    int countIntersections()
    {
        return left + right + top + bottom;
    }

    void printIntersections(void);

    // Of the stored intersections, this returns the one closest to the
    // specified point
    void nearest(double x, double y, double& xi, double& yi);
};

}  // namespace vpsc
