///
/// @file ANode.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __A_NODE_HXX_74A52539A1CE__
#define __A_NODE_HXX_74A52539A1CE__

namespace avoid {

class VertInf;

class ANode
{
public:
    VertInf* inf;
    double   g;        // Gone
    double   h;        // Heuristic
    double   f;        // Formula f = g + h

    ANode* prevNode;   // VertInf for the previous ANode.
    int    timeStamp;  // Time-stamp used to determine exploration order of
                       // seemingly equal paths during orthogonal routing.

    ANode(VertInf* vinf, int time)
        : inf(vinf)
        , g(0)
        , h(0)
        , f(0)
        , prevNode(nullptr)
        , timeStamp(time)
    {
    }

    ANode()
        : inf(nullptr)
        , g(0)
        , h(0)
        , f(0)
        , prevNode(nullptr)
        , timeStamp(-1)
    {
    }
};

}  // namespace avoid

#endif  // __A_NODE_HXX_74A52539A1CE__
