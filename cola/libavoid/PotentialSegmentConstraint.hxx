///
/// @file PotentialSegmentConstraint.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __POTENTIAL_SEGMENT_CONSTRAINT_HXX_E8CD685D690B__
#define __POTENTIAL_SEGMENT_CONSTRAINT_HXX_E8CD685D690B__

namespace avoid {

class PotentialSegmentConstraint
{
public:
    PotentialSegmentConstraint(
        size_t           index1,
        size_t           index2,
        const Variables& vs
    )
        : index1(index1)
        , index2(index2)
        , vs(vs)
    {
    }

    bool operator<(const PotentialSegmentConstraint rhs) const
    {
        return sepDistance() < rhs.sepDistance();
    }

    double sepDistance(void) const
    {
        if (!stillValid())
        {
            return 0;
        }
        return fabs(vs[index1]->finalPosition - vs[index2]->finalPosition);
    }

    bool stillValid(void) const
    {
        return (index1 != index2);
    }

    void rewriteIndex(size_t oldIndex, size_t newIndex)
    {
        if (index1 == oldIndex)
        {
            index1 = newIndex;
        }

        if (index2 == oldIndex)
        {
            index2 = newIndex;
        }
    }

    size_t index1;
    size_t index2;

private:
    const Variables& vs;
};

}  // namespace avoid

#endif  // __POTENTIAL_SEGMENT_CONSTRAINT_HXX_E8CD685D690B__
