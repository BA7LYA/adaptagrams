///
/// @file ConnDirFlag.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace avoid {

//! @brief  Flags that can be passed to the ConnEnd constructor to specify
//!         which sides of a shape this point should have visibility to if
//!         it is located within the shape's area.
//!
//! Like SVG, libavoid considers the Y-axis to point downwards, that is,
//! like screen coordinates the coordinates increase from left-to-right and
//! also from top-to-bottom.
//!
enum ConnDirFlag
{
    ConnDirNone = 0,

    //! @brief  This option specifies the point should be given visibility
    //!         to the top of the shape that it is located within.
    ConnDirUp = 1,

    //! @brief  This option specifies the point should be given visibility
    //!         to the bottom of the shape that it is located within.
    ConnDirDown = 2,

    //! @brief  This option specifies the point should be given visibility
    //!         to the left side of the shape that it is located within.
    ConnDirLeft = 4,

    //! @brief  This option specifies the point should be given visibility
    //!         to the right side of the shape that it is located within.
    ConnDirRight = 8,

    //! @brief  This option, provided for convenience, specifies the point
    //!         should be given visibility to all four sides of the shape
    //!         that it is located within.
    ConnDirAll = 15
};

//! @brief  One or more avoid::ConnDirFlag options.
//!
using ConnDirFlags = unsigned int;

}  // namespace avoid
