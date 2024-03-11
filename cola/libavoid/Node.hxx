///
/// @file Node.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __NODE_HXX_97934D5EB59E__
#define __NODE_HXX_97934D5EB59E__

namespace avoid {

class Obstacle;
class ShiftSegment;
class VertInf;

class Node
{
public:
    Obstacle*         v;
    VertInf*          c;
    ShiftSegment*     ss;
    double            pos;
    double            min[2], max[2];
    Node *            firstAbove, *firstBelow;
    NodeSet::iterator iter;

    Node(Obstacle* v, const double p);
    Node(VertInf* c, const double p);
    Node(ShiftSegment* ss, const double p);
    virtual ~Node();

    double firstObstacleAbove(size_t dim);
    double firstObstacleBelow(size_t dim);
    void   markShiftSegmentsAbove(size_t dim);
    void   markShiftSegmentsBelow(size_t dim);
    void   findFirstPointAboveAndBelow(
          const size_t dim,
          const double linePos,
          double&      firstAbovePos,
          double&      firstBelowPos,
          double&      lastAbovePos,
          double&      lastBelowPos
      );
    double firstPointAbove(size_t dim);
    double firstPointBelow(size_t dim);
    bool   isInsideShape(size_t dimension);
};

}  // namespace avoid

#endif  // __NODE_HXX_97934D5EB59E__
