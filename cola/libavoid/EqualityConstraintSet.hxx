///
/// @file EqualityConstraintSet.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __EQUALITY_CONSTRAINT_SET_HXX_588522AD214A__
#define __EQUALITY_CONSTRAINT_SET_HXX_588522AD214A__

namespace avoid {

class EqualityConstraintSet
{
public:
    EqualityConstraintSet(Variables vs)
    {
        for (size_t i = 0; i < vs.size(); ++i)
        {
            std::map<Variable*, double> varSet;
            varSet[vs[i]] = 0;
            variableGroups.push_back(varSet);
        }
    }

    bool isRedundant(Variable* lhs, Variable* rhs, double sep)
    {
        VarOffsetMapList::iterator lhsSet = setForVar(lhs);
        VarOffsetMapList::iterator rhsSet = setForVar(rhs);
        if (lhsSet == rhsSet)
        {
            // Check if this is a redundant constraint.
            if (fabs(((*lhsSet)[lhs] + sep) - (*rhsSet)[rhs]) < 0.0001)
            {
                // If so, return true.
                return true;
            }
        }
        return false;
    }

    void mergeSets(Variable* lhs, Variable* rhs, double sep)
    {
        VarOffsetMapList::iterator lhsSet = setForVar(lhs);
        VarOffsetMapList::iterator rhsSet = setForVar(rhs);
        if (lhsSet == rhsSet)
        {
            return;
        }

        double rhsOldOffset = (*rhsSet)[rhs];
        double rhsNewOffset = (*lhsSet)[lhs] + sep;
        double offset       = rhsNewOffset - rhsOldOffset;

        // Update offsets
        for (std::map<Variable*, double>::iterator it = rhsSet->begin();
             it != rhsSet->end();
             ++it)
        {
            it->second += offset;
        }

        // Merge rhsSet into lhsSet
        lhsSet->insert(rhsSet->begin(), rhsSet->end());
        variableGroups.erase(rhsSet);
    }

private:
    VarOffsetMapList::iterator setForVar(Variable* var)
    {
        for (VarOffsetMapList::iterator it = variableGroups.begin();
             it != variableGroups.end();
             ++it)
        {
            if (it->find(var) != it->end())
            {
                return it;
            }
        }
        return variableGroups.end();
    }

    VarOffsetMapList variableGroups;
};

}  // namespace avoid

#endif  // __EQUALITY_CONSTRAINT_SET_HXX_588522AD214A__
