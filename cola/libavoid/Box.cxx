///
/// @file Box.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/Box.hxx"

namespace avoid {

double Box::length(size_t dimension) const
{
    if (dimension == 0)
    {
        return (max.x - min.x);
    }
    return (max.y - min.y);
}

double Box::width(void) const
{
    return (max.x - min.x);
}

double Box::height(void) const
{
    return (max.y - min.y);
}

}  // namespace avoid
