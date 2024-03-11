///
/// @file LineRep.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include <list>

#include "libavoid/Point.hxx"

namespace avoid {

// LineReps: Used for highlighting certain areas in debugging output.
struct LineRep
{
    Point begin;
    Point end;
};

using LineReps = std::list<LineRep>;

}  // namespace avoid
