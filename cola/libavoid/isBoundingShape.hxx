///
/// @file isBoundingShape.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __IS_BOUNDING_SHAPE_HXX_A516B781ACD2__
#define __IS_BOUNDING_SHAPE_HXX_A516B781ACD2__

namespace avoid {

class isBoundingShape
{
public:
    // Class instance remembers the ShapeSet.
    isBoundingShape(ShapeSet& set)
        : ss(set)
    {
    }

    // The following is an overloading of the function call operator.
    bool operator()(const PointPair& pp)
    {
        if (!(pp.vInf->id.isConnPt())
            && (ss.find(pp.vInf->id.objID) != ss.end()))
        {
            return true;
        }
        return false;
    }

private:
    // MSVC wants to generate the assignment operator and the default
    // constructor, but fails.  Therefore we declare them private and
    // don't implement them.
    isBoundingShape& operator=(const isBoundingShape&);
    isBoundingShape();

    ShapeSet& ss;
};

}  // namespace avoid

#endif  // __IS_BOUNDING_SHAPE_HXX_A516B781ACD2__
