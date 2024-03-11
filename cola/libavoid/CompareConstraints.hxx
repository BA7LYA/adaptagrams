///
/// @file CompareConstraints.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __COMPARE_CONSTRAINTS_HXX_2C3F0A952368__
#define __COMPARE_CONSTRAINTS_HXX_2C3F0A952368__

namespace avoid {

class Constraint;

class CompareConstraints
{
public:
    bool operator()(Constraint* const& l, Constraint* const& r) const;
};

}  // namespace avoid

#endif  // __COMPARE_CONSTRAINTS_HXX_2C3F0A952368__
