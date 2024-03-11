///
/// @file HeapCmpVertInf.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __HEAP_CMP_VERTINF_HXX_DA097BC10B6C__
#define __HEAP_CMP_VERTINF_HXX_DA097BC10B6C__

namespace avoid {

// Comparison for the vertex heap in the extended Dijkstra's algorithm.
struct HeapCmpVertInf
{
    bool operator()(const VertInf* a, const VertInf* b) const;
};

}  // namespace avoid

#endif  // __HEAP_CMP_VERTINF_HXX_DA097BC10B6C__
