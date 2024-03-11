///
/// @file Variable.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __VARIABLE_HXX_C8B40115F131__
#define __VARIABLE_HXX_C8B40115F131__

namespace avoid {

class Variable
{
    friend std::ostream& operator<<(std::ostream& os, const Variable& v);
    friend class Block;
    friend class Constraint;
    friend class IncSolver;

public:
    int         id;      // useful in log files
    double      desiredPosition;
    double      finalPosition;
    double      weight;  // how much the variable wants to
                         // be at it's desired position
    double      scale;   // translates variable to another space
    double      offset;
    Block*      block;
    bool        visited;
    bool        fixedDesiredPosition;
    Constraints in;
    Constraints out;

    inline Variable(
        const int    id,
        const double desiredPos = -1.0,
        const double weight     = 1.0,
        const double scale      = 1.0
    )
        : id(id)
        , desiredPosition(desiredPos)
        , weight(weight)
        , scale(scale)
        , offset(0)
        , block(nullptr)
        , visited(false)
        , fixedDesiredPosition(false)
    {
    }

    double dfdv(void) const
    {
        return 2. * weight * (position() - desiredPosition);
    }

private:
    inline double position(void) const
    {
        return (block->ps.scale * block->posn + offset) / scale;
    }

    inline double unscaledPosition(void) const
    {
        COLA_ASSERT(block->ps.scale == 1);
        COLA_ASSERT(scale == 1);
        return block->posn + offset;
    }
};

}  // namespace avoid

#endif  // __VARIABLE_HXX_C8B40115F131__
