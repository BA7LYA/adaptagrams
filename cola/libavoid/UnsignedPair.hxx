///
/// @file UnsignedPair.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __UNSIGNED_PAIR_HXX_8EB114834B59__
#define __UNSIGNED_PAIR_HXX_8EB114834B59__

#include <set>

namespace avoid {

// A pair of unsigned values that can be compared and used as the keys
// for sets and maps.
class UnsignedPair
{
public:
    UnsignedPair(unsigned ind1, unsigned ind2)
    {
        COLA_ASSERT(ind1 != ind2);

        // Assign the lesser value to m_index1.
        m_index1 = (ind1 < ind2) ? ind1 : ind2;

        // Assign the greater value to m_index2.
        m_index2 = (ind1 > ind2) ? ind1 : ind2;
    }

    bool operator<(const UnsignedPair& rhs) const
    {
        if (m_index1 != rhs.m_index1)
        {
            return m_index1 < rhs.m_index1;
        }
        return m_index2 < rhs.m_index2;
    }

private:
    unsigned short m_index1;
    unsigned short m_index2;
};

typedef std::set<UnsignedPair> UnsignedPairSet;

}  // namespace avoid

#endif  // __UNSIGNED_PAIR_HXX_8EB114834B59__
