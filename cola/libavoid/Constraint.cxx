///
/// @file Constraint.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/Constraint.hxx"

namespace avoid {

Constraint::Constraint(
    Variable* left,
    Variable* right,
    double    gap,
    bool      equality
)
    : left(left)
    , right(right)
    , gap(gap)
    , timeStamp(0)
    , active(false)
    , equality(equality)
    , unsatisfiable(false)
    , needsScaling(true)
    , creator(nullptr)
{
    // In hindsight I think it's probably better to build the constraint DAG
    // (by creating variable in/out lists) when needed, rather than in advance
    // left->out.push_back(this);
    // right->in.push_back(this);
}

Constraint::~Constraint()
{
    // see constructor: the following is just way too slow.
    // Better to create a
    // new DAG on demand than maintain the lists dynamically.
    // Constraints::iterator i;
    // for(i=left->out.begin(); i!=left->out.end(); i++) {
    // if(*i==this) break;
    //}
    // left->out.erase(i);
    // for(i=right->in.begin(); i!=right->in.end(); i++) {
    // if(*i==this) break;
    //}
    // right->in.erase(i);
}

std::string Constraint::toString(void) const
{
    std::stringstream stream;
    stream << "Constraint: var(" << left->id << ") ";
    if (gap < 0)
    {
        stream << "- " << -gap << " ";
    }
    else
    {
        stream << "+ " << gap << " ";
    }
    stream << ((equality) ? "==" : "<=");
    stream << " var(" << right->id << ") ";
    return stream.str();
}

}  // namespace avoid
