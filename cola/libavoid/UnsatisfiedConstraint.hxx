///
/// @file UnsatisfiedConstraint.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __UNSATISFIED_CONSTRAINT_HXX_503E27191062__
#define __UNSATISFIED_CONSTRAINT_HXX_503E27191062__

namespace avoid {

class Constraint;

struct UnsatisfiedConstraint
{
    UnsatisfiedConstraint(Constraint& c)
        : c(c)
    {
    }

    Constraint& c;
};

}  // namespace avoid

#endif  // __UNSATISFIED_CONSTRAINT_HXX_503E27191062__
