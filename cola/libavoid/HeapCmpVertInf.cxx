///
/// @file HeapCmpVertInf.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/HeapCmpVertInf.hxx"

namespace avoid {

// Comparison for the vertex heap in the extended Dijkstra's algorithm.
bool HeapCmpVertInf::operator()(const VertInf* a, const VertInf* b) const
{
    return a->sptfDist > b->sptfDist;
}

}  // namespace avoid
