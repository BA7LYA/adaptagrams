///
/// @file Polygon.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include <vector>

#include "libavoid/PolygonInterface.hxx"

namespace avoid {

//! @brief  A dynamic Polygon, to which points can be easily added and removed.
//!
//! @note The Rectangle class can be used as an easy way of constructing a
//!       square or rectangular polygon.
//!
class AVOID_EXPORT Polygon : public PolygonInterface
{
public:
    //! @brief  Constructs an empty polygon (with zero points).
    Polygon();

    //! @brief  Constructs a new polygon with n points.
    //!
    //! A rectangle would be comprised of four point.  An n segment
    //! PolyLine (represented as a Polygon) would be comprised of n+1
    //! points.  Whether a particular Polygon is closed or not, depends
    //! on whether it is a Polygon or Polyline.  Shape polygons are always
    //! considered to be closed, meaning the last point joins back to the
    //! first point.
    //!
    //! The values for points can be set by setting the Polygon:ps vector,
    //! or via the Polygon::setPoint() method.
    //!
    //! @param[in]  n  Number of points in the polygon.
    //!
    Polygon(const int n);

    //! @brief  Constructs a new polygon from an existing Polygon.
    //!
    //! @param[in]  poly  An existing polygon to copy the new polygon from.
    //!
    Polygon(const PolygonInterface& poly);

    //! @brief  Resets this to the empty polygon.
    void clear(void);

    //! @brief  Returns true if this polygon is empty.
    bool empty(void) const;

    //! @brief  Returns the number of points in this polygon.
    size_t size(void) const;

    //! @brief  Returns the ID value associated with this polygon.
    int id(void) const;

    //! @brief  Returns a specific point in the polygon.
    //! @param[in]  index  The array index of the point to be returned.
    const Point& at(size_t index) const;

    //! @brief  Sets a position for a particular point in the polygon..
    //! @param[in]  index  The array index of the point to be set.
    //! @param[in]  point  The point value to be assigned..
    void setPoint(size_t index, const Point& point);

    //! @brief  Returns a simplified Polyline, where all collinear line
    //!         segments have been collapsed down into single line
    //!         segments.
    //!
    //! @return A new polyline with a simplified representation.
    //!
    Polygon simplify(void) const;

    //! @brief  Returns a curved approximation of this multi-segment
    //!         PolyLine, with the corners replaced by smooth Bezier
    //!         curves.
    //!
    //! This function does not do any further obstacle avoidance with the
    //! curves produced.  Hence, you would usually specify a curve_amount
    //! in similar size to the space buffer around obstacles in the scene.
    //! This way the curves will cut the corners around shapes but still
    //! run within this buffer space.
    //!
    //! @param  curve_amount  Describes the distance along the end of each
    //!                       line segment to turn into a curve.
    //! @param  closed        Describes whether the Polygon should be
    //!                       treated as closed.  Defaults to false.
    //! @return A new polyline (polygon) representing the curved path.
    //!         Its points represent endpoints of line segments and
    //!         Bezier spline control points.  The Polygon::ts vector for
    //!         this returned polygon is populated with a character for
    //!         each point describing its type.
    //! @sa     ts
    Polygon curvedPolyline(const double curve_amount, const bool closed = false)
        const;

    //! @brief  Translates the polygon position by a relative amount.
    //!
    //! @param[in]  xDist  Distance to move polygon in the x dimension.
    //! @param[in]  yDist  Distance to move polygon in the y dimension.
    void translate(const double xDist, const double yDist);

    //! @brief  An ID for the polygon.
    int _id;

    //! @brief  A vector of the points that make up the Polygon.
    std::vector<Point> ps;

    //! @brief  If used, denotes whether the corresponding point in ps is
    //!         a move-to operation or a Bezier curve-to.
    //!
    //! Each character describes the drawing operation for the
    //! corresponding point in the ps vector.  Possible values are:
    //!  -  'M': A moveto operation, marks the first point;
    //!  -  'L': A lineto operation, is a line from the previous point to
    //!     the current point; or
    //!  -  'C': A curveto operation, three consecutive 'C' points
    //!     (along with the previous point) describe the control points
    //!     of a Bezier curve.
    //!  -  'Z': Closes the path (used for cluster boundaries).
    //!
    //! @note   This vector will currently only be populated for polygons
    //!         returned by curvedPolyline().
    std::vector<char> ts;

    // @brief  If used, denotes checkpoints through which the route travels
    //         and the relevant segment of the route.
    //
    // Set and used by the orthogonal routing code. Note the first value
    // in the pair doesn't correspond to the segment index containing the
    // checkpoint, but rather the segment or bendpoint on which it lies.
    //    0 if on ps[0]
    //    1 if on line ps[0]-ps[1]
    //    2 if on ps[1]
    //    3 if on line ps[1]-ps[2]
    //    etc.
    std::vector<std::pair<size_t, Point>> checkpointsOnRoute;

    // Returns true if at least one checkpoint lies on the line segment
    // or at either end of it.  An indexModifier of +1 will cause it to
    // ignore a checkpoint on the corner at the start of the segment and
    // -1 will cause it to do the same for the corner at the end of the
    // segment.
    std::vector<Point> checkpointsOnSegment(
        size_t segmentLowerIndex,
        int    indexModifier = 0
    ) const;
};

}  // namespace avoid
