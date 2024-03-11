///
/// @file DesiredPosition.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace cola {

/*
 * Setting a desired position for a node adds a term to the goal function
 * drawing the node towards that desired position
 */
struct DesiredPosition
{
    unsigned id;
    double   x;
    double   y;
    double   weight;
};

using DesiredPositions = std::vector<cola::DesiredPosition>;

}  // namespace cola
