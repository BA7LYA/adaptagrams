///
/// @file CompareConstraints.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/CompareConstraints.hxx"

namespace avoid {

bool CompareConstraints::operator()(Constraint* const& l, Constraint* const& r)
    const
{
    const double sl = l->left->block->timeStamp > l->timeStamp
                           || l->left->block == l->right->block
                        ? -DBL_MAX
                        : l->slack();
    const double sr = r->left->block->timeStamp > r->timeStamp
                           || r->left->block == r->right->block
                        ? -DBL_MAX
                        : r->slack();
    if (sl == sr)
    {
        // arbitrary choice based on id
        if (l->left->id == r->left->id)
        {
            if (l->right->id < r->right->id)
            {
                return true;
            }
            return false;
        }
        if (l->left->id < r->left->id)
        {
            return true;
        }
        return false;
    }
    return sl > sr;
}

}  // namespace avoid
