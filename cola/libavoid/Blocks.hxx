///
/// @file Blocks.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __BLOCKS_HXX_1AB8D1803541__
#define __BLOCKS_HXX_1AB8D1803541__

#include <list>
#include <vector>

namespace avoid {

class Constraint;
class Block;
class Variable;
class Variables;

/*
 * A block structure defined over the variables such that each block contains
 * 1 or more variables, with the invariant that all constraints inside a block
 * are satisfied by keeping the variables fixed relative to one another
 */
class Blocks
{
public:
    Blocks(const Variables& vs);
    ~Blocks(void);

    void                  mergeLeft(Block* r);
    void                  mergeRight(Block* l);
    void                  split(Block* b, Block*& l, Block*& r, Constraint* c);
    std::list<Variable*>* totalOrder();
    void                  cleanup();
    double                cost();

    size_t size() const;
    Block* at(size_t index) const;
    void   insert(Block* block);

    long blockTimeCtr;

private:
    void dfsVisit(Variable* v, std::list<Variable*>* order);
    void removeBlock(Block* doomed);

    std::vector<Block*> m_blocks;
    const Variables&    vs;
    size_t              nvs;
};

}  // namespace avoid

#endif  // __BLOCKS_HXX_1AB8D1803541__
