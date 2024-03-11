///
/// @file PositionStats.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __POSITION_STATS_HXX_8BEEEC0B8DBD__
#define __POSITION_STATS_HXX_8BEEEC0B8DBD__

namespace avoid {

class Variable;

struct PositionStats
{
    PositionStats()
        : scale(0)
        , AB(0)
        , AD(0)
        , A2(0)
    {
    }

    void   addVariable(Variable* const v);
    double scale;
    double AB;
    double AD;
    double A2;
};

}  // namespace avoid

#endif  // __POSITION_STATS_HXX_8BEEEC0B8DBD__
