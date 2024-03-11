///
/// @file PolygonInterface.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/PolygonInterface.hxx"

#include <algorithm>

#include "libavoid/Box.hxx"
#include "libavoid/Point.hxx"
#include "libavoid/Polygon.hxx"
#include "libavoid/Rectangle.hxx"

namespace avoid {

Polygon PolygonInterface::offsetPolygon(double offset) const
{
    Polygon newPoly;
    newPoly._id = id();
    if (offset == 0)
    {
        for (size_t i = 0; i < size(); ++i)
        {
            newPoly.ps.push_back(at(i));
        }
        return newPoly;
    }

    size_t              numOfEdges = size();
    std::vector<Vector> normals(numOfEdges);
    for (size_t i = 0; i < numOfEdges; ++i)
    {
        normals[i] = unitNormalForEdge(at(i), at((i + 1) % numOfEdges));
    }

    size_t j = numOfEdges - 1;
    for (size_t i = 0; i < numOfEdges; ++i)
    {
        double R
            = 1
            + ((normals[i].x * normals[j].x) + (normals[i].y * normals[j].y));
        if (((normals[j].x * normals[i].y) - (normals[i].x * normals[j].y))
                * offset
            >= 0)
        {
            double q  = offset / R;
            Point  pt = Point(
                at(i).x + (normals[j].x + normals[i].x) * q,
                at(i).y + (normals[j].y + normals[i].y) * q
            );

            pt.id = id();
            pt.vn = newPoly.size();
            newPoly.ps.push_back(pt);
        }
        else
        {
            Point pt1 = Point(
                at(i).x + normals[j].x * offset,
                at(i).y + normals[j].y * offset
            );
            Point pt2 = at(i);
            Point pt3 = Point(
                at(i).x + normals[i].x * offset,
                at(i).y + normals[i].y * offset
            );

            pt1.id = id();
            pt1.vn = newPoly.size();
            newPoly.ps.push_back(pt1);

            pt2.id = id();
            pt2.vn = newPoly.size();
            newPoly.ps.push_back(pt2);

            pt3.id = id();
            pt3.vn = newPoly.size();
            newPoly.ps.push_back(pt3);
        }
        j = i;
    }

    return newPoly;
}

Polygon PolygonInterface::boundingRectPolygon(void) const
{
    Box boundingBox = offsetBoundingBox(0.0);

    return Rectangle(boundingBox.min, boundingBox.max);
}

Box PolygonInterface::offsetBoundingBox(double offset) const
{
    Box bBox;

    bBox.min.x = DBL_MAX;
    bBox.min.y = DBL_MAX;
    bBox.max.x = -DBL_MAX;
    bBox.max.y = -DBL_MAX;

    for (size_t i = 0; i < size(); ++i)
    {
        bBox.min.x = std::min(bBox.min.x, at(i).x);
        bBox.min.y = std::min(bBox.min.y, at(i).y);
        bBox.max.x = std::max(bBox.max.x, at(i).x);
        bBox.max.y = std::max(bBox.max.y, at(i).y);
    }

    // Add buffer space.
    bBox.min.x -= offset;
    bBox.min.y -= offset;
    bBox.max.x += offset;
    bBox.max.y += offset;

    return bBox;
}

}  // namespace avoid
