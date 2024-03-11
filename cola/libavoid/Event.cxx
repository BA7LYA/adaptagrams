///
/// @file Event.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/Event.hxx"

namespace avoid {

Event::Event(EventType t, Node* v, double p)
    : type(t)
    , v(v)
    , pos(p)
{
}

}  // namespace avoid
