///
/// @file CmpVisEdgeRotation.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __CMP_VIS_EDGE_ROTATION_HXX_0DA5A4098E46__
#define __CMP_VIS_EDGE_ROTATION_HXX_0DA5A4098E46__

namespace avoid {

class CmpVisEdgeRotation
{
public:
    CmpVisEdgeRotation(const VertInf* lastPt)
        : _lastPt(lastPt)
    {
    }

    bool operator()(const EdgeInf* u, const EdgeInf* v) const
    {
        // Dummy ShapeConnectionPin edges are not orthogonal and
        // therefore can't be compared in the same way.
        if (u->isOrthogonal() && v->isOrthogonal())
        {
            return u->rotationLessThan(_lastPt, v);
        }
        return u < v;
    }

private:
    const VertInf* _lastPt;
};

}  // namespace avoid

#endif  // __CMP_VIS_EDGE_ROTATION_HXX_0DA5A4098E46__
