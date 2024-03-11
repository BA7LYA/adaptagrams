///
/// @file CmpIndexes.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __CMP_INDEXES_HXX_0DF3EBBFC204__
#define __CMP_INDEXES_HXX_0DF3EBBFC204__

namespace avoid {

// Used to sort points when merging NudgingShiftSegments.
// Sorts the indexes, by point position in one dimension.
class CmpIndexes
{
public:
    CmpIndexes(ConnRef* conn, size_t dim)
        : connRef(conn)
        , dimension(dim)
    {
    }

    bool operator()(size_t lhs, size_t rhs)
    {
        return connRef->displayRoute().ps[lhs][dimension]
             < connRef->displayRoute().ps[rhs][dimension];
    }

private:
    ConnRef* connRef;
    size_t   dimension;
};

}  // namespace avoid

#endif  // __CMP_INDEXES_HXX_0DF3EBBFC204__
