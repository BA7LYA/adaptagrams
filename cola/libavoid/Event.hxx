///
/// @file Event.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __EVENT_HXX_10CEFAA5289C__
#define __EVENT_HXX_10CEFAA5289C__

#include "libavoid/EventType.hxx"

namespace avoid {

class Node;

struct Event
{
    Event(EventType t, Node* v, double p);

    EventType type;
    Node*     v;
    double    pos;
};

}  // namespace avoid

#endif  // __EVENT_HXX_10CEFAA5289C__
