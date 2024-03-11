///
/// @file CmpNodePos.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/CmpNodePos.hxx"

#include "libavoid/Node.hxx"

namespace avoid {

bool CmpNodePos::operator()(const Node* u, const Node* v) const
{
    if (u->pos != v->pos)
    {
        return u->pos < v->pos;
    }

    // Use the pointers to the base objects to differentiate them.
    void* up = (u->v) ? (void*)u->v : ((u->c) ? (void*)u->c : (void*)u->ss);
    void* vp = (v->v) ? (void*)v->v : ((v->c) ? (void*)v->c : (void*)v->ss);

    return up < vp;
}

}  // namespace avoid
