///
/// @file ConnEndType.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace avoid {

//! @brief  Types that describe the kind a connection that a ConnEnd
//!         represents.
//!
enum ConnEndType
{
    //! @brief  The ConnEnd represents a free-floating point that may or may
    //! not have visibility in specific directions.
    ConnEndPoint,

    //! @brief  The ConnEnd attaches to a specific ShapeConnectionPin on a
    //! shape.
    ConnEndShapePin,

    //! @brief  The ConnEnd attaches to a junction.
    ConnEndJunction,

    //! @brief  The ConnEnd is empty and doesn't have any information set.
    ConnEndEmpty
};

}  // namespace avoid
