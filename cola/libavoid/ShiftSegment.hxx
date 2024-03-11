///
/// @file ShiftSegment.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __SHIFT_SEGMENT_HXX_BF2AEA24F513__
#define __SHIFT_SEGMENT_HXX_BF2AEA24F513__

namespace avoid {

// ShiftSegment interface.
class ShiftSegment
{
public:
    ShiftSegment(const size_t dim)
        : dimension(dim)
    {
    }

    virtual ~ShiftSegment() {}

    virtual Point&       lowPoint(void)        = 0;
    virtual Point&       highPoint(void)       = 0;
    virtual const Point& lowPoint(void) const  = 0;
    virtual const Point& highPoint(void) const = 0;
    virtual bool overlapsWith(const ShiftSegment* rhs, const size_t dim) const
        = 0;
    virtual bool immovable(void) const = 0;

    size_t dimension;
    double minSpaceLimit;
    double maxSpaceLimit;
};

using ShiftSegmentList = std::list<ShiftSegment*>;

}  // namespace avoid

#endif  // __SHIFT_SEGMENT_HXX_BF2AEA24F513__
