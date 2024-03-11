///
/// @file CmpEdgeInf.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/CmpEdgeInf.hxx"

namespace avoid {

// Comparison for the bridging edge heap in the extended Kruskal's algorithm.
bool CmpEdgeInf::operator()(const EdgeInf* a, const EdgeInf* b) const
{
    return a->mtstDist() > b->mtstDist();
}

}  // namespace avoid
