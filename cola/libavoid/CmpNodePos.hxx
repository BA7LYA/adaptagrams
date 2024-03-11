///
/// @file CmpNodePos.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __CMP_NODE_POS_HXX_6EF36C4CBA17__
#define __CMP_NODE_POS_HXX_6EF36C4CBA17__

#include <set>

namespace avoid {

class Node;

struct CmpNodePos
{
    bool operator()(const Node* u, const Node* v) const;
};

using NodeSet = std::set<Node*, CmpNodePos>;

}  // namespace avoid

#endif  // __CMP_NODE_POS_HXX_6EF36C4CBA17__
