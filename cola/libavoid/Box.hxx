///
/// @file Box.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/dllexport.hxx"

namespace avoid {

//! @brief  A bounding box, represented by the top-left and
//!         bottom-right corners.
//!
class AVOID_EXPORT Box
{
public:
    //! The top-left point.
    Point min;

    //! The bottom-right point.
    Point max;

    double length(size_t dimension) const;
    double width(void) const;
    double height(void) const;
};

}  // namespace avoid
