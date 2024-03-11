///
/// @file Point.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/dllexport.hxx"

namespace avoid {

//! @brief  The Point class defines a point in the plane.
//!
//! Points consist of an x and y value.  They may also have an ID and vertex
//! number associated with them.
//!
class AVOID_EXPORT Point
{
public:
    //! @brief  Default constructor.
    //!
    Point();

    //! @brief  Standard constructor.
    //!
    //! @param[in]  xv  The x position of the point.
    //! @param[in]  yv  The y position of the point.
    //!
    Point(const double xv, const double yv);

    //! @brief  Comparison operator. Returns true if at same position.
    //!
    //! @param[in]  rhs  The point to compare with this one.
    //! @return          The result of the comparison.
    //! @sa         equals()
    bool operator==(const Point& rhs) const;

    //! @brief  Comparison operator. Returns true if at same position,
    //!         or at effectively the same position for a given value of
    //!         epsilson.
    //!
    //! @param[in]  rhs      The point to compare with this one.
    //! @param[in]  epsilon  Value of epsilon to use during comparison.
    //! @return              The result of the comparison.
    //! @sa         operator==()
    bool equals(const Point& rhs, double epsilon = 0.0001) const;

    //! @brief  Comparison operator. Returns true if at different positions.
    //!
    //! @param[in]  rhs  The point to compare with this one.
    //! @return          The result of the comparison.
    //!
    bool operator!=(const Point& rhs) const;

    //! @brief  Comparison operator. Returns true if less-then rhs point.
    //!
    //! @note  This operator is not particularly useful, but is defined
    //!        to allow std::set<Point>.
    //!
    //! @param[in]  rhs  The point to compare with this one.
    //! @return          The result of the comparison.
    //!
    bool operator<(const Point& rhs) const;

    //! @brief  Returns the x or y value of the point, given the dimension.
    //!
    //! @param[in]  dimension  The dimension:  0 for x, 1 for y.
    //! @return                The component of the point in that dimension.
    double&       operator[](const size_t dimension);
    const double& operator[](const size_t dimension) const;

    Point operator+(const Point& rhs) const;
    Point operator-(const Point& rhs) const;

    //! The x position.
    double         x;
    //! The y position.
    double         y;
    //! The ID associated with this point.
    unsigned int   id;
    //! The vertex number associated with this point.
    unsigned short vn;
};

}  // namespace avoid
