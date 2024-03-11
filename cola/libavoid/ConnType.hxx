///
/// @file ConnType.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace avoid {

//! @brief  Describes the type of routing that is performed for each
//!         connector.
enum ConnType
{
    ConnType_None       = 0,
    //! @brief  The connector path will be a shortest-path poly-line that
    //!         routes around obstacles.
    ConnType_PolyLine   = 1,
    //! @brief  The connector path will be a shortest-path orthogonal
    //!         poly-line (only vertical and horizontal line segments) that
    //!         routes around obstacles.
    ConnType_Orthogonal = 2
};

}  // namespace avoid
