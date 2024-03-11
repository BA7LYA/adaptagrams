///
/// @file CmpEdgeInf.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __CMP_EDGE_INF_HXX_EAFCDE494EC3__
#define __CMP_EDGE_INF_HXX_EAFCDE494EC3__

namespace avoid {

// Comparison for the bridging edge heap in the extended Kruskal's algorithm.
struct CmpEdgeInf
{
    bool operator()(const EdgeInf* a, const EdgeInf* b) const;
};

}  // namespace avoid

#endif  // __CMP_EDGE_INF_HXX_EAFCDE494EC3__
