///
/// @file Block.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __BLOCK_HXX_6DDEADA3C2E8__
#define __BLOCK_HXX_6DDEADA3C2E8__

namespace avoid {

class Block
{
    typedef Variables::iterator         Vit;
    typedef Constraints::iterator       Cit;
    typedef Constraints::const_iterator Cit_const;

    friend std::ostream& operator<<(std::ostream& os, const Block& b);

public:
    Variables*    vars;
    double        posn;
    // double weight;
    // double wposn;
    PositionStats ps;

    Block(Blocks* blocks, Variable* const v = nullptr);
    ~Block(void);

    Constraint* findMinLM();
    Constraint* findMinLMBetween(Variable* const lv, Variable* const rv);
    Constraint* findMinInConstraint();
    Constraint* findMinOutConstraint();
    void        deleteMinInConstraint();
    void        deleteMinOutConstraint();
    void        updateWeightedPosition();
    void        merge(Block* b, Constraint* c, double dist);
    Block*      merge(Block* b, Constraint* c);
    void        mergeIn(Block* b);
    void        mergeOut(Block* b);
    void        split(Block*& l, Block*& r, Constraint* c);
    Constraint* splitBetween(
        Variable* vl,
        Variable* vr,
        Block*&   lb,
        Block*&   rb
    );
    void   setUpInConstraints();
    void   setUpOutConstraints();
    double cost();
    bool   deleted;
    long   timeStamp;
    Heap*  in;
    Heap*  out;
    bool   getActivePathBetween(
          Constraints&    path,
          const Variable* u,
          const Variable* v,
          const Variable* w
      ) const;
    bool isActiveDirectedPathBetween(const Variable* u, const Variable* v)
        const;
    bool getActiveDirectedPathBetween(
        Constraints&    path,
        const Variable* u,
        const Variable* v
    ) const;

private:
    typedef enum
    {
        NONE,
        LEFT,
        RIGHT
    } Direction;

    typedef std::pair<double, Constraint*> Pair;
    void   reset_active_lm(Variable* const v, Variable* const u);
    void   list_active(Variable* const v, Variable* const u);
    double compute_dfdv(Variable* const v, Variable* const u);
    double compute_dfdv(
        Variable* const v,
        Variable* const u,
        Constraint*&    min_lm
    );
    bool split_path(
        Variable*,
        Variable* const,
        Variable* const,
        Constraint*& min_lm,
        bool         desperation
    );
    bool canFollowLeft(const Constraint* c, const Variable* last) const;
    bool canFollowRight(const Constraint* c, const Variable* last) const;
    void populateSplitBlock(Block* b, Variable* v, const Variable* u);
    void addVariable(Variable* v);
    void setUpConstraintHeap(Heap*& h, bool in);

    // Parent container, that holds the blockTimeCtr.
    Blocks* blocks;
};

}  // namespace avoid

#endif  // __BLOCK_HXX_6DDEADA3C2E8__
