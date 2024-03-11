///
/// @file SegmentListWrapper.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __SEGMENT_LIST_WRAPPER_HXX_61EA62B132CE__
#define __SEGMENT_LIST_WRAPPER_HXX_61EA62B132CE__

namespace avoid {

class SegmentListWrapper
{
public:
    LineSegment* insert(LineSegment segment)
    {
        SegmentList::iterator found = _list.end();
        for (SegmentList::iterator curr = _list.begin(); curr != _list.end();
             ++curr)
        {
            if (curr->overlaps(segment))
            {
                if (found != _list.end())
                {
                    // This is not the first segment that overlaps,
                    // so we need to merge and then delete an existing
                    // segment.
                    curr->mergeVertInfs(*found);
                    _list.erase(found);
                    found = curr;
                }
                else
                {
                    // This is the first overlapping segment, so just
                    // merge the new segment with this one.
                    curr->mergeVertInfs(segment);
                    found = curr;
                }
            }
        }

        if (found == _list.end())
        {
            // Add this line.
            _list.push_back(segment);
            return &(_list.back());
        }

        return &(*found);
    }

    SegmentList& list(void)
    {
        return _list;
    }

private:
    SegmentList _list;
};

}  // namespace avoid

#endif  // __SEGMENT_LIST_WRAPPER_HXX_61EA62B132CE__
