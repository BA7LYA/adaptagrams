///
/// @file Constraint.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __CONSTRAINT_HXX_6C9E0E28EDD2__
#define __CONSTRAINT_HXX_6C9E0E28EDD2__

namespace avoid {

class Constraint
{
public:
    Constraint(
        Variable* left,
        Variable* right,
        double    gap,
        bool      equality = false
    );
    ~Constraint();

    inline double slack(void) const
    {
        if (unsatisfiable)
        {
            return DBL_MAX;
        }
        if (needsScaling)
        {
            return right->scale * right->position() - gap
                 - left->scale * left->position();
        }
        COLA_ASSERT(left->scale == 1);
        COLA_ASSERT(right->scale == 1);
        return right->unscaledPosition() - gap - left->unscaledPosition();
    }

    std::string toString(void) const;

    friend std::ostream& operator<<(std::ostream& os, const Constraint& c);
    Variable*            left;
    Variable*            right;
    double               gap;
    double               lm;
    long                 timeStamp;
    bool                 active;
    const bool           equality;
    bool                 unsatisfiable;
    bool                 needsScaling;
    void*                creator;
};

}  // namespace avoid

#endif  // __CONSTRAINT_HXX_6C9E0E28EDD2__
