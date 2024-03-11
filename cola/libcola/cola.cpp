/*
 * vim: ts=4 sw=4 et tw=0 wm=0
 *
 * libcola - A library providing force-directed network layout using the
 *           stress-majorization method subject to separation constraints.
 *
 * Copyright (C) 2006-2014  Monash University
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * See the file LICENSE.LGPL distributed with the library.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author(s):  Tim Dwyer
 */

#include "libcola/cola.h"

#include <cmath>

#include "libcola/cluster.h"
#include "libcola/commondefs.h"
#include "libcola/conjugate_gradient.h"
#include "libcola/shortest_paths.h"
#include "libcola/straightener.h"
#include "libvpsc/assertions.h"

using namespace std;
using namespace vpsc;
using straightener::generateClusterBoundaries;

namespace cola {

Rectangle bounds(vector<Rectangle*>& rs)
{
    COLA_ASSERT(!rs.empty());

    double left = rs[0]->getMinX(), right = rs[0]->getMaxX(),
           top = rs[0]->getMinY(), bottom = rs[0]->getMaxY();

    for (unsigned i = 1; i < rs.size(); i++)
    {
        left   = min(left, rs[i]->getMinX());
        right  = max(right, rs[i]->getMaxX());
        top    = min(top, rs[i]->getMinY());
        bottom = max(bottom, rs[i]->getMaxY());
    }
    return Rectangle(left, right, top, bottom);
}

#if 0
    void removeClusterOverlap(RootCluster& clusterHierarchy, vpsc::Rectangles& rs, Locks& locks, vpsc::Dim dim) {
        if(clusterHierarchy.nodes.size()>0 || clusterHierarchy.clusters.size()>0) {
            vpsc::Variables vars;
            vpsc::Constraints cs;
            for(unsigned i=0;i<rs.size();i++) {
                vars.push_back(new vpsc::Variable(i, rs[i]->getCentreD(dim)));
            }
            
            clusterHierarchy.computeBoundingRect(rs);
            clusterHierarchy.createVars(dim,rs,vars);
            clusterHierarchy.generateNonOverlapConstraints(dim, cola::Both, rs, vars, cs);
            
            /*
            if(dim==vpsc::HORIZONTAL) {
                vpsc::Rectangle::setXBorder(0.001);
                // use rs->size() rather than n because some of the variables may
                // be dummy vars with no corresponding rectangle
                generateXConstraints(rs,vars,cs,true); 
                vpsc::Rectangle::setXBorder(0);
            } else {
                generateYConstraints(rs,vars,cs); 
            }
            */
            for(Locks::iterator l=locks.begin();
                    l!=locks.end();l++) {
                unsigned id=l->getID();
                double x=l->pos(HORIZONTAL), y=l->pos(VERTICAL);
                Variable* v=vars[id];
                v->desiredPosition = (dim==vpsc::HORIZONTAL)?x:y;
                v->weight = 1000;
            }
            /*
            vpsc::Solver s(vars,cs);
            try {
                s.satisfy();
            } catch(const char* e) {
                cerr << "ERROR from solver in GraphData::removeOverlap : " << e << endl;
            }
            */
            vpsc::IncSolver s(vars,cs);
            try {
                s.solve();
            } catch(const char* e) {
                cerr << "ERROR from solver in GraphData::removeOverlap : " << e << endl;
            }
            clusterHierarchy.updateBounds(dim);
            /*
            for(unsigned i=0;i<cs.size();++i) {
                if(cs[i]->unsatisfiable) {
                    cout << "Unsatisfiable constraint: " << *cs[i] << endl;
                }
            }
            */
            for(unsigned i=0;i<rs.size();i++) {
                rs[i]->moveCentreD(dim,vars[i]->finalPosition);
            }
            for(Locks::iterator l=locks.begin();
                    l!=locks.end();l++) {
                //unsigned id=l->getID();
            }
            for_each(vars.begin(),vars.end(),delete_object());
            for_each(cs.begin(),cs.end(),delete_object());
        }
    }
    void removeClusterOverlapFast(RootCluster& clusterHierarchy, vpsc::Rectangles& rs, Locks& locks) {
        removeClusterOverlap(clusterHierarchy, rs, locks, vpsc::HORIZONTAL);
        removeClusterOverlap(clusterHierarchy, rs, locks, vpsc::VERTICAL);
    }
#endif

ConstrainedMajorizationLayout* simpleCMLFactory(
    vpsc::Rectangles&        rs,
    const std::vector<Edge>& es,
    RootCluster*             clusterHierarchy,
    const double             idealLength,
    bool                     useNeighbourStress
)
{
    cola::EdgeLengths eLengths;
    for (size_t i = 0; i < es.size(); ++i)
    {
        eLengths.push_back(1);
    }
    return new ConstrainedMajorizationLayout(
        rs,
        es,
        clusterHierarchy,
        idealLength,
        eLengths,
        nullptr,
        nullptr,
        useNeighbourStress
    );
};

}  // namespace cola
