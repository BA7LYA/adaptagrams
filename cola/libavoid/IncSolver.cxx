///
/// @file IncSolver.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/IncSolver.hxx"

namespace avoid {

IncSolver::IncSolver(const Variables& vs, const Constraints& cs)
    : m(cs.size())
    , cs(cs)
    , n(vs.size())
    , vs(vs)
    , needsScaling(false)
{
    for (unsigned i = 0; i < n; ++i)
    {
        vs[i]->in.clear();
        vs[i]->out.clear();

        // Set needsScaling if any variables have a scale other than 1.
        needsScaling |= (vs[i]->scale != 1);
    }
    for (unsigned i = 0; i < m; ++i)
    {
        Constraint* c = cs[i];
        c->left->out.push_back(c);
        c->right->in.push_back(c);
        c->needsScaling = needsScaling;
    }
    bs = new Blocks(vs);
#ifdef LIBVPSC_LOGGING
    printBlocks();
    // COLA_ASSERT(!constraintGraphIsCyclic(n,vs));
#endif

    inactive = cs;
    for (Constraints::iterator i = inactive.begin(); i != inactive.end(); ++i)
    {
        (*i)->active = false;
    }
}

IncSolver::~IncSolver()
{
    delete bs;
}

void IncSolver::addConstraint(Constraint* c)
{
    ++m;
    c->active = false;
    inactive.push_back(c);
    c->left->out.push_back(c);
    c->right->in.push_back(c);
    c->needsScaling = needsScaling;
}

// useful in debugging
void IncSolver::printBlocks()
{
#ifdef LIBVPSC_LOGGING
    ofstream f(LOGFILE, ios::app);
    for (set<Block*>::iterator i = bs->begin(); i != bs->end(); ++i)
    {
        Block* b = *i;
        f << "  " << *b << endl;
    }
    for (unsigned i = 0; i < m; i++)
    {
        f << "  " << *cs[i] << endl;
    }
#endif
}

/*
 * Stores the relative positions of the variables in their finalPosition
 * field.
 */
void IncSolver::copyResult()
{
    for (Variables::const_iterator i = vs.begin(); i != vs.end(); ++i)
    {
        Variable* v      = *i;
        v->finalPosition = v->position();
        COLA_ASSERT(v->finalPosition == v->finalPosition);
    }
}

// useful in debugging - cycles would be BAD
bool IncSolver::constraintGraphIsCyclic(const unsigned n, Variable* const vs[])
{
    map<Variable*, node*> varmap;
    vector<node*>         graph;
    for (unsigned i = 0; i < n; i++)
    {
        node* u = new node;
        graph.push_back(u);
        varmap[vs[i]] = u;
    }
    for (unsigned i = 0; i < n; i++)
    {
        for (vector<Constraint*>::iterator c = vs[i]->in.begin();
             c != vs[i]->in.end();
             ++c)
        {
            Variable* l = (*c)->left;
            varmap[vs[i]]->in.insert(varmap[l]);
        }

        for (vector<Constraint*>::iterator c = vs[i]->out.begin();
             c != vs[i]->out.end();
             ++c)
        {
            Variable* r = (*c)->right;
            varmap[vs[i]]->out.insert(varmap[r]);
        }
    }
    while (graph.size() > 0)
    {
        node*                   u = nullptr;
        vector<node*>::iterator i = graph.begin();
        for (; i != graph.end(); ++i)
        {
            u = *i;
            if (u->in.size() == 0)
            {
                break;
            }
        }
        if (i == graph.end() && graph.size() > 0)
        {
            // cycle found!
            return true;
        }
        else
        {
            graph.erase(i);
            for (set<node*>::iterator j = u->out.begin(); j != u->out.end();
                 ++j)
            {
                node* v = *j;
                v->in.erase(u);
            }
            delete u;
        }
    }
    for (unsigned i = 0; i < graph.size(); ++i)
    {
        delete graph[i];
    }
    return false;
}

// useful in debugging - cycles would be BAD
bool IncSolver::blockGraphIsCyclic()
{
    map<Block*, node*> bmap;
    vector<node*>      graph;
    size_t             length = bs->size();
    for (size_t i = 0; i < length; ++i)
    {
        Block* b = bs->at(i);
        node*  u = new node;
        graph.push_back(u);
        bmap[b] = u;
    }
    for (size_t i = 0; i < length; ++i)
    {
        Block* b = bs->at(i);
        b->setUpInConstraints();
        Constraint* c = b->findMinInConstraint();
        while (c != nullptr)
        {
            Block* l = c->left->block;
            bmap[b]->in.insert(bmap[l]);
            b->deleteMinInConstraint();
            c = b->findMinInConstraint();
        }

        b->setUpOutConstraints();
        c = b->findMinOutConstraint();
        while (c != nullptr)
        {
            Block* r = c->right->block;
            bmap[b]->out.insert(bmap[r]);
            b->deleteMinOutConstraint();
            c = b->findMinOutConstraint();
        }
    }
    while (graph.size() > 0)
    {
        node*                   u = nullptr;
        vector<node*>::iterator i = graph.begin();
        for (; i != graph.end(); ++i)
        {
            u = *i;
            if (u->in.size() == 0)
            {
                break;
            }
        }
        if (i == graph.end() && graph.size() > 0)
        {
            // cycle found!
            return true;
        }
        else
        {
            graph.erase(i);
            for (set<node*>::iterator j = u->out.begin(); j != u->out.end();
                 ++j)
            {
                node* v = *j;
                v->in.erase(u);
            }
            delete u;
        }
    }
    for (unsigned i = 0; i < graph.size(); i++)
    {
        delete graph[i];
    }
    return false;
}

bool IncSolver::solve()
{
#ifdef LIBVPSC_LOGGING
    ofstream f(LOGFILE, ios::app);
    f << "solve_inc()..." << endl;
#endif
    satisfy();
    double lastcost = DBL_MAX, cost = bs->cost();
    while (fabs(lastcost - cost) > 0.0001)
    {
        satisfy();
        lastcost = cost;
        cost     = bs->cost();
#ifdef LIBVPSC_LOGGING
        f << "  bs->size=" << bs->size() << ", cost=" << cost << endl;
#endif
    }
    copyResult();
    return bs->size() != n;
}

/*
 * incremental version of satisfy that allows refinement after blocks are
 * moved.
 *
 *  - move blocks to new positions
 *  - repeatedly merge across most violated constraint until no more
 *    violated constraints exist
 *
 * Note: there is a special case to handle when the most violated constraint
 * is between two variables in the same block.  Then, we must split the block
 * over an active constraint between the two variables.  We choose the
 * constraint with the most negative lagrangian multiplier.
 */
bool IncSolver::satisfy()
{
#ifdef LIBVPSC_LOGGING
    ofstream f(LOGFILE, ios::app);
    f << "satisfy_inc()..." << endl;
#endif
    splitBlocks();
    // long splitCtr = 0;
    Constraint* v = nullptr;
    // CBuffer buffer(inactive);
    while ((v = mostViolated(inactive))
           && (v->equality || ((v->slack() < ZERO_UPPERBOUND) && !v->active)))
    {
        COLA_ASSERT(!v->active);
        Block *lb = v->left->block, *rb = v->right->block;
        if (lb != rb)
        {
            lb->merge(rb, v);
        }
        else
        {
            if (lb->isActiveDirectedPathBetween(v->right, v->left))
            {
                // cycle found, relax the violated, cyclic constraint
                v->unsatisfiable = true;
                continue;
                // UnsatisfiableException e;
                // lb->getActiveDirectedPathBetween(e.path,v->right,v->left);
                // e.path.push_back(v);
                // throw e;
            }
            // if(splitCtr++>10000) {
            // throw "Cycle Error!";
            //}
            // constraint is within block, need to split first
            try
            {
                Constraint* splitConstraint
                    = lb->splitBetween(v->left, v->right, lb, rb);
                if (splitConstraint != nullptr)
                {
                    COLA_ASSERT(!splitConstraint->active);
                    inactive.push_back(splitConstraint);
                }
                else
                {
                    v->unsatisfiable = true;
                    continue;
                }
            }
            catch (UnsatisfiableException e)
            {
                e.path.push_back(v);
                /*
                std::cerr << "Unsatisfiable:" << std::endl;
                for(std::vector<Constraint*>::iterator r=e.path.begin();
                        r!=e.path.end();++r)
                {
                    std::cerr << **r <<std::endl;
                }
                */
                v->unsatisfiable = true;
                continue;
            }
            if (v->slack() >= 0)
            {
                COLA_ASSERT(!v->active);
                // v was satisfied by the above split!
                inactive.push_back(v);
                bs->insert(lb);
                bs->insert(rb);
            }
            else
            {
                bs->insert(lb->merge(rb, v));
                delete ((lb->deleted) ? lb : rb);
            }
        }
#ifdef LIBVPSC_LOGGING
        f << "...remaining blocks=" << bs->size() << ", cost=" << bs->cost()
          << endl;
#endif
    }
#ifdef LIBVPSC_LOGGING
    f << "  finished merges." << endl;
#endif
    bs->cleanup();
    bool activeConstraints = false;
    for (unsigned i = 0; i < m; i++)
    {
        v = cs[i];
        if (v->active)
        {
            activeConstraints = true;
        }
        if (v->slack() < ZERO_UPPERBOUND)
        {
            ostringstream s;
            s << "Unsatisfied constraint: " << *v;
#ifdef LIBVPSC_LOGGING
            ofstream f(LOGFILE, ios::app);
            f << s.str() << endl;
#endif
            throw s.str().c_str();
        }
    }
#ifdef LIBVPSC_LOGGING
    f << "  finished cleanup." << endl;
    printBlocks();
#endif
    copyResult();
    return activeConstraints;
}

void IncSolver::moveBlocks()
{
#ifdef LIBVPSC_LOGGING
    ofstream f(LOGFILE, ios::app);
    f << "moveBlocks()..." << endl;
#endif
    size_t length = bs->size();
    for (size_t i = 0; i < length; ++i)
    {
        Block* b = bs->at(i);
        b->updateWeightedPosition();
        // b->posn = b->wposn / b->weight;
    }
#ifdef LIBVPSC_LOGGING
    f << "  moved blocks." << endl;
#endif
}

void IncSolver::splitBlocks()
{
#ifdef LIBVPSC_LOGGING
    ofstream f(LOGFILE, ios::app);
#endif
    moveBlocks();
    splitCnt      = 0;
    // Split each block if necessary on min LM
    size_t length = bs->size();
    for (size_t i = 0; i < length; ++i)
    {
        Block*      b = bs->at(i);
        Constraint* v = b->findMinLM();
        if (v != nullptr && v->lm < LAGRANGIAN_TOLERANCE)
        {
            COLA_ASSERT(!v->equality);
#ifdef LIBVPSC_LOGGING
            f << "    found split point: " << *v << " lm=" << v->lm << endl;
#endif
            splitCnt++;
            Block *b = v->left->block, *l = nullptr, *r = nullptr;
            COLA_ASSERT(v->left->block == v->right->block);
            // double pos = b->posn;
            b->split(l, r, v);
            // l->posn=r->posn=pos;
            // l->wposn = l->posn * l->weight;
            // r->wposn = r->posn * r->weight;
            l->updateWeightedPosition();
            r->updateWeightedPosition();
            bs->insert(l);
            bs->insert(r);
            b->deleted = true;
            COLA_ASSERT(!v->active);
            inactive.push_back(v);
#ifdef LIBVPSC_LOGGING
            f << "  new blocks: " << *l << " and " << *r << endl;
#endif
        }
    }
    // if(splitCnt>0) { std::cout<<"  splits: "<<splitCnt<<endl; }
#ifdef LIBVPSC_LOGGING
    f << "  finished splits." << endl;
#endif
    bs->cleanup();
}

/*
 * Scan constraint list for the most violated constraint, or the first equality
 * constraint
 */
Constraint* IncSolver::mostViolated(Constraints& l)
{
    double      slackForMostViolated = DBL_MAX;
    Constraint* mostViolated         = nullptr;
#ifdef LIBVPSC_LOGGING
    ofstream f(LOGFILE, ios::app);
    f << "Looking for most violated..." << endl;
#endif
    size_t      lSize       = l.size();
    size_t      deleteIndex = lSize;
    Constraint* constraint  = nullptr;
    double      slack       = 0;
    for (size_t index = 0; index < lSize; ++index)
    {
        constraint = l[index];
        slack      = constraint->slack();
        if (constraint->equality || slack < slackForMostViolated)
        {
            slackForMostViolated = slack;
            mostViolated         = constraint;
            deleteIndex          = index;
            if (constraint->equality)
            {
                break;
            }
        }
    }
    // Because the constraint list is not order dependent we just
    // move the last element over the deletePoint and resize
    // downwards.  There is always at least 1 element in the
    // vector because of search.
    if ((deleteIndex < lSize)
        && (((slackForMostViolated < ZERO_UPPERBOUND) && !mostViolated->active)
            || mostViolated->equality))
    {
        l[deleteIndex] = l[lSize - 1];
        l.resize(lSize - 1);
    }
#ifdef LIBVPSC_LOGGING
    if (mostViolated)
    {
        f << "  most violated is: " << *mostViolated << endl;
    }
    else
    {
        f << "  non found." << endl;
    }
#endif
    return mostViolated;
}

}  // namespace avoid
