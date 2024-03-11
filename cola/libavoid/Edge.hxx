///
/// @file Edge.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/dllexport.hxx"

namespace avoid {

//! @brief  A line between two points.
//!
class AVOID_EXPORT Edge
{
public:
    //! The first point.
    Point a;

    //! The second point.
    Point b;
};

}  // namespace avoid
