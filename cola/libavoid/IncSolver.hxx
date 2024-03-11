///
/// @file IncSolver.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __INC_SOLVER_HXX_3190B40738FF__
#define __INC_SOLVER_HXX_3190B40738FF__

namespace avoid {

/*
 * Variable Placement with Separation Constraints problem instance
 */
class IncSolver
{
public:
    unsigned splitCnt;
    bool     satisfy();
    bool     solve();
    void     moveBlocks();
    void     splitBlocks();
    IncSolver(const Variables& vs, const Constraints& cs);

    ~IncSolver();
    void addConstraint(Constraint* constraint);

    const Variables& getVariables()
    {
        return vs;
    }

protected:
    Blocks*            bs;
    size_t             m;
    const Constraints& cs;
    size_t             n;
    const Variables&   vs;
    bool               needsScaling;

    void printBlocks();
    void copyResult();

private:
    bool        constraintGraphIsCyclic(const unsigned n, Variable* const vs[]);
    bool        blockGraphIsCyclic();
    Constraints inactive;
    Constraints violated;
    Constraint* mostViolated(Constraints& l);
};

}  // namespace avoid

#endif  // __INC_SOLVER_HXX_3190B40738FF__
