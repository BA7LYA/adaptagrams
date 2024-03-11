///
/// @file Point.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/Point.hxx"

namespace avoid {

Point::Point()
    : id(0)
    , vn(kUnassignedVertexNumber)
{
}

Point::Point(const double xv, const double yv)
    : x(xv)
    , y(yv)
    , id(0)
    , vn(kUnassignedVertexNumber)
{
}

bool Point::operator==(const Point& rhs) const
{
    if ((x == rhs.x) && (y == rhs.y))
    {
        return true;
    }
    return false;
}

bool Point::operator!=(const Point& rhs) const
{
    if ((x != rhs.x) || (y != rhs.y))
    {
        return true;
    }
    return false;
}

bool Point::equals(const Point& rhs, double epsilon) const
{
    if ((fabs(x - rhs.x) < epsilon) && (fabs(y - rhs.y) < epsilon))
    {
        return true;
    }
    return false;
}

// Just defined to allow std::set<Point>.  Not particularly meaningful!
bool Point::operator<(const Point& rhs) const
{
    if (x == rhs.x)
    {
        return (y < rhs.y);
    }
    return (x < rhs.x);
}

double& Point::operator[](const size_t dimension)
{
    COLA_ASSERT((dimension == 0) || (dimension == 1));
    return ((dimension == 0) ? x : y);
}

const double& Point::operator[](const size_t dimension) const
{
    COLA_ASSERT((dimension == 0) || (dimension == 1));
    return ((dimension == 0) ? x : y);
}

Point Point::operator+(const Point& rhs) const
{
    return Point(x + rhs.x, y + rhs.y);
}

Point Point::operator-(const Point& rhs) const
{
    return Point(x - rhs.x, y - rhs.y);
}

}  // namespace avoid
