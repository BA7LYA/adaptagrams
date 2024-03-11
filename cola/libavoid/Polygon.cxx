///
/// @file Polygon.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/Polygon.hxx"

namespace avoid {

Polygon::Polygon()
    : PolygonInterface()
    , _id(0)
{
    clear();
}

Polygon::Polygon(const int pn)
    : PolygonInterface()
    , _id(0)
    , ps(pn)
{
}

Polygon::Polygon(const PolygonInterface& poly)
    : PolygonInterface()
    , _id(poly.id())
    , ps(poly.size())
{
    for (size_t i = 0; i < poly.size(); ++i)
    {
        ps[i] = poly.at(i);
    }
}

void Polygon::clear(void)
{
    ps.clear();
    ts.clear();
}

bool Polygon::empty(void) const
{
    return ps.empty();
}

size_t Polygon::size(void) const
{
    return ps.size();
}

int Polygon::id(void) const
{
    return _id;
}

const Point& Polygon::at(size_t index) const
{
    COLA_ASSERT(index < size());

    return ps[index];
}

void Polygon::setPoint(size_t index, const Point& point)
{
    COLA_ASSERT(index < size());

    ps[index] = point;
}

void Polygon::translate(const double xDist, const double yDist)
{
    for (size_t i = 0; i < size(); ++i)
    {
        ps[i].x += xDist;
        ps[i].y += yDist;
    }
}

Polygon Polygon::simplify(void) const
{
    // Copy the PolyLine.
    Polygon simplified = *this;

    std::vector<std::pair<size_t, Point>>& checkpoints
        = simplified.checkpointsOnRoute;
    bool hasCheckpointInfo = !(checkpoints.empty());

    std::vector<Point>::iterator it = simplified.ps.begin();
    if (it != simplified.ps.end())
    {
        ++it;
    }

    // Combine collinear line segments into single segments:
    for (size_t j = 2; j < simplified.size();)
    {
        if (vecDir(simplified.ps[j - 2], simplified.ps[j - 1], simplified.ps[j])
            == 0)
        {
            // These three points make up two collinear segments, so just
            // combine them into a single segment.
            it = simplified.ps.erase(it);

            if (hasCheckpointInfo)
            {
                // 0     1     2     3     4   <- vertices on path
                // +-----+-----+-----+-----+
                // 0  1  2  3  4  5  6  7  8   <- checkpoints on points & edges
                //             |
                //             \_ deletedPointValue = 4
                //
                // If 1-2-3 is collinear then we want to end up with
                //
                // 0     1           2     3
                // +-----+-----------+-----+
                // 0  1  2  3  3  3  4  5  6
                //
                //
                //
                size_t deletedPointValue = (j - 1) - 1;
                for (size_t i = 0; i < checkpoints.size(); ++i)
                {
                    if (checkpoints[i].first == deletedPointValue)
                    {
                        checkpoints[i].first -= 1;
                    }
                    else if (checkpoints[i].first > deletedPointValue)
                    {
                        checkpoints[i].first -= 2;
                    }
                }
            }
        }
        else
        {
            ++j;
            ++it;
        }
    }

    return simplified;
}

std::vector<Point> Polygon::checkpointsOnSegment(
    size_t segmentLowerIndex,
    int    indexModifier
) const
{
    std::vector<Point> checkpoints;
    // 0     1     2     3     4   <- vertices on path
    // +-----+-----+-----+-----+
    // 0  1  2  3  4  5  6  7  8   <- checkpoints on points & edges

    size_t checkpointLowerValue = 2 * segmentLowerIndex;
    size_t checkpointUpperValue = checkpointLowerValue + 2;
    size_t index                = 0;

    if (indexModifier > 0)
    {
        checkpointLowerValue++;
    }
    else if (indexModifier < 0)
    {
        checkpointUpperValue--;
    }

    while (index < checkpointsOnRoute.size())
    {
        if ((checkpointsOnRoute[index].first >= checkpointLowerValue)
            && (checkpointsOnRoute[index].first <= checkpointUpperValue))
        {
            checkpoints.push_back(checkpointsOnRoute[index].second);
        }
        ++index;
    }
    return checkpoints;
}

// curvedPolyline():
//     Returns a curved approximation of this multi-segment PolyLine, with
//     the corners replaced by smooth Bezier curves.  curve_amount describes
//     how large to make the curves.
//     The ts value for each point in the returned Polygon describes the
//     drawing operation: 'M' (move) marks the first point, a line segment
//     is marked with an 'L' and three 'C's (along with the previous point)
//     describe the control points of a Bezier curve.
//
Polygon Polygon::curvedPolyline(const double curve_amount, const bool closed)
    const
{
    Polygon simplified = this->simplify();

    Polygon curved;
    size_t  num_of_points = size();
    if (num_of_points <= 2)
    {
        // There is only a single segment, do nothing.
        curved = *this;
        curved.ts.push_back('M');
        curved.ts.push_back('L');
        return curved;
    }

    // Build the curved polyline:
    curved._id    = _id;
    double last_x = 0;
    double last_y = 0;
    if (closed)
    {
        double x1 = simplified.ps[0].x;
        double y1 = simplified.ps[0].y;
        double x2 = simplified.ps[1].x;
        double y2 = simplified.ps[1].y;
        shorten_line(x1, y1, x2, y2, SHORTEN_START, curve_amount);
        curved.ps.push_back(Point(x1, y1));
        curved.ts.push_back('M');
    }
    else
    {
        curved.ps.push_back(ps[0]);
        curved.ts.push_back('M');
    }

    size_t simpSize = simplified.size();
    size_t finish   = (closed) ? simpSize + 2 : simpSize;
    for (size_t j = 1; j < finish; ++j)
    {
        double x1 = simplified.ps[(simpSize + j - 1) % simpSize].x;
        double y1 = simplified.ps[(simpSize + j - 1) % simpSize].y;
        double x2 = simplified.ps[j % simpSize].x;
        double y2 = simplified.ps[j % simpSize].y;

        double old_x = x1;
        double old_y = y1;

        unsigned int mode = SHORTEN_BOTH;
        if (!closed)
        {
            if (j == 1)
            {
                mode = SHORTEN_END;
            }
            else if (j == (size() - 1))
            {
                mode = SHORTEN_START;
            }
        }
        shorten_line(x1, y1, x2, y2, mode, curve_amount);

        if (j > 1)
        {
            curved.ts.insert(curved.ts.end(), 3, 'C');
            curved.ps.push_back(Point(mid(last_x, old_x), mid(last_y, old_y)));
            curved.ps.push_back(Point(mid(x1, old_x), mid(y1, old_y)));
            curved.ps.push_back(Point(x1, y1));
        }
        if (closed && (j == (finish - 1)))
        {
            // Close the path.
            curved.ts.push_back('Z');
            curved.ps.push_back(Point(x1, y1));
            break;
        }
        curved.ts.push_back('L');
        curved.ps.push_back(Point(x2, y2));

        last_x = x2;
        last_y = y2;
    }

    return curved;
}

}  // namespace avoid
