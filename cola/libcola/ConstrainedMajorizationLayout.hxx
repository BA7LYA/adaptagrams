///
/// @file ConstrainedMajorizationLayout.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include <vector>

namespace cola {

/**
 * @brief  Implements the Constrained Majorization graph layout algorithm
 *         (deprecated).
 *
 * The optimisation method used is "stress majorization", where a sequence of
 * quadratic functions which strictly bound the stress from above, is solved
 * to monotonically reduce the stress (by iteratively changing the
 * configuration of nodes).
 *
 * Once the problem has been set up, call run() or runOnce() to run the
 * layout algorithm.
 *
 * @note  We recommend the use of ConstrainedFDLayout instead of this class.
 *        ConstrainedFDLayout tends to produce better results and has more
 *        features.  We are no longer working on ConstrainedMajorizationLayout.
 */
class ConstrainedMajorizationLayout
{
public:
    /**
     * @brief Constructs a constrained majorization layout instance.
     *
     * @param[in] rs  Bounding boxes of nodes at their initial positions.
     * @param[in] es  Simple pair edges, giving indices of the start and end
     *                nodes in rs.
     * @param[in] clusterHierarchy  A pointer to a RootCluster object defining
     *                              the cluster hierarchy (optional).
     * @param[in] idealLength  Aa scalar modifier of ideal edge lengths in
     *                         eLengths.
     * @param[in] eLengths  Individual ideal lengths for edges.
     *                      The actual ideal length used for the ith edge is
     *                      idealLength*eLengths[i], or if eLengths is empty
     *                      then just idealLength is used (i.e., eLengths[i]
     *                      is assumed to be 1).
     * @param[in] done  A test of convergence operation called at the end of
     *                  each iteration (optional).
     * @param[in] preIteration  An operation called before each iteration
     *                          (optional).
     */
    ConstrainedMajorizationLayout(
        vpsc::Rectangles&        rs,
        const std::vector<Edge>& es,
        RootCluster*             clusterHierarchy,
        const double             idealLength,
        EdgeLengths              eLengths           = StandardEdgeLengths,
        TestConvergence*         doneTest           = nullptr,
        PreIteration*            preIteration       = nullptr,
        bool                     useNeighbourStress = false
    );

    /**
     * @brief  Specify a set of compound constraints to apply to the layout.
     *
     * @param[in] ccs  The compound constraints.
     */
    void setConstraints(cola::CompoundConstraints* ccs)
    {
        constrainedLayout = true;
        this->ccs         = ccs;
    }

    void setConstraintsVector(cola::CompoundConstraints& ccs)
    {
        constrainedLayout               = true;
        cola::CompoundConstraints* ccsp = new cola::CompoundConstraints;
        for (size_t i = 0; i < ccs.size(); ++i)
        {
            ccsp->push_back(ccs.at(i));
        }
        this->ccs = ccsp;
    }

    /**
     * @brief Register to receive information about unsatisfiable constraints.
     *
     * In the case of unsatisifiable constraints, the solver will drop
     * unsatisfiable constraints of lowest priority.  Information about these
     * will be written to these lists after each iteration of constrained
     * layout.
     *
     * param[out] unsatisfiableX A pointer to an UnsatisfiableConstraintInfos
     *                           object that will be used to record
     *                           unsatisfiable constraints in the x-dimension.
     * param[out] unsatisfiableY A pointer to an UnsatisfiableConstraintInfos
     *                           object that will be used to record
     *                           unsatisfiable constraints in the y-dimension.
     */
    void setUnsatisfiableConstraintInfo(
        UnsatisfiableConstraintInfos* unsatisfiableX,
        UnsatisfiableConstraintInfos* unsatisfiableY
    )
    {
        this->unsatisfiableX = unsatisfiableX;
        this->unsatisfiableY = unsatisfiableY;
    }

    /**
     * Sticky nodes causes nodes to spring back to (startX,startY) when
     * unconstrained.
     */
    void setStickyNodes(
        const double                 stickyWeight,
        const std::valarray<double>& startX,
        const std::valarray<double>& startY
    );

    /**
     * Scaling speeds up the solver by conditioning the quadratic terms matrix.
     */
    void setScaling(bool scaling)
    {
        this->scaling = scaling;
    }

    /**
     * Says that the Mosek optimisation library should be used to solve the
     * quadratic programs rather than the libvpsc solver.
     */
    void setExternalSolver(bool externalSolver)
    {
        this->externalSolver = externalSolver;
    }

    /**
     * At each iteration of layout, generate constraints to avoid overlaps.
     * If bool horizontal is true, all overlaps will be resolved horizontally,
     * otherwise some overlaps will be left to be resolved vertically where
     * doing so leads to less displacement
     */
    void setAvoidOverlaps(bool horizontal = false)
    {
        constrainedLayout   = true;
        this->avoidOverlaps = horizontal ? Horizontal : Both;
    }

    /**
     * Add constraints to prevent clusters overlapping.
     */
    void setNonOverlappingClusters()
    {
        constrainedLayout      = true;
        nonOverlappingClusters = true;
    }

    /**
     * For the specified edges (with routings), generate dummy vars and
     * constraints to try and straighten them.  bendWeight controls how hard we
     * try to straighten existing bends potBendWeight controls how much we try
     * to keep straight edges straight
     */
    void setStraightenEdges(
        std::vector<straightener::Edge*>* straightenEdges,
        double                            bendWeight    = 0.01,
        double                            potBendWeight = 0.1,
        bool                              xSkipping     = true
    )
    {
        for (std::vector<straightener::Edge*>::const_iterator e
             = straightenEdges->begin();
             e != straightenEdges->end();
             e++)
        {
            (*e)->rerouteAround(boundingBoxes);
        }
        constrainedLayout     = true;
        this->xSkipping       = xSkipping;
        this->straightenEdges = straightenEdges;
        this->bendWeight      = bendWeight;
        this->potBendWeight   = potBendWeight;
    }

    /**
     * Update position of bounding boxes.
     */
    void moveBoundingBoxes()
    {
        for (unsigned i = 0; i < n; i++)
        {
            boundingBoxes[i]->moveCentre(X[i], Y[i]);
        }
    }

    ~ConstrainedMajorizationLayout()
    {
        if (using_default_done)
        {
            delete done;
        }

        if (constrainedLayout)
        {
            delete gpX;
            delete gpY;
        }
    }

    /**
     * @brief  Implements the main layout loop, taking descent steps until
     *         stress is no-longer significantly reduced.
     *
     * @param[in] x  If true, layout will be performed in x-dimension
     *               (default: true).
     * @param[in] y  If true, layout will be performed in y-dimension
     *               (default: true).
     */
    void run(bool x = true, bool y = true);

    /**
     * @brief  Same as run(), but only applies one iteration.
     *
     * This may be useful here it's too hard to implement a call-back
     * (e.g., in java apps).
     *
     * @param[in] x  If true, layout will be performed in x-dimension
     *               (default: true).
     * @param[in] y  If true, layout will be performed in y-dimension
     *               (default: true).
     */
    void runOnce(bool x = true, bool y = true);

    void straighten(std::vector<straightener::Edge*>&, vpsc::Dim);

    void setConstrainedLayout(bool c)
    {
        constrainedLayout = c;
    }

    double computeStress();

private:
    double euclidean_distance(unsigned i, unsigned j)
    {
        return sqrt(
            (X[i] - X[j]) * (X[i] - X[j]) + (Y[i] - Y[j]) * (Y[i] - Y[j])
        );
    }

    double compute_stress(const std::valarray<double>& Dij);
    void   majorize(
          const std::valarray<double>& Dij,
          GradientProjection*          gp,
          std::valarray<double>&       coords,
          const std::valarray<double>& startCoords
      );
    void newton(
        const std::valarray<double>& Dij,
        GradientProjection*          gp,
        std::valarray<double>&       coords,
        const std::valarray<double>& startCoords
    );

    unsigned n;  //< number of nodes

    // std::valarray<double> degrees;
    std::valarray<double> lap2;  //< graph laplacian
    std::valarray<double> Q;     //< quadratic terms matrix used in computations
    std::valarray<double> Dij;   //< all pairs shortest path distances
    double                tol;   //< convergence tolerance
    TestConvergence* done;  //< functor used to determine if layout is finished
    bool using_default_done;  // Whether we allocated a default TestConvergence
                              // object.
    PreIteration*
        preIteration;         //< client can use this to create locks on nodes
    vpsc::Rectangles boundingBoxes;  //< node bounding boxes

    /*
     * stickyNodes controls whether nodes are attracted to their starting
     * positions (at time of ConstrainedMajorizationLayout instantiation)
     * stored in startX, startY
     */
    std::valarray<double> X, Y;
    bool                  stickyNodes;
    double                stickyWeight;
    std::valarray<double> startX;
    std::valarray<double> startY;
    double                edge_length;
    bool                  constrainedLayout;
    bool                  nonOverlappingClusters;

    /*
     * A cluster is a set of nodes that are somehow semantically grouped
     * and should therefore be kept together a bit more tightly than, and
     * preferably without overlapping, the rest of the graph.
     *
     * We achieve this by augmenting the L matrix with stronger attractive
     * forces between all members of a cluster (other than the root)
     * and by maintaining a (preferably convex) hull around those
     * constituents which, using constraints and dummy variables, is
     * prevented from overlapping other parts of the graph.
     *
     * Clusters are defined over the graph in a hierarchy starting with
     * a single root cluster.
     *
     * Need to:
     *  - augment Lap matrix with intra cluster forces
     *  - compute convex hull of each cluster
     *  - from convex hull generate "StraightenEdges"
     */
    RootCluster*                      clusterHierarchy;
    GradientProjection *              gpX, *gpY;
    cola::CompoundConstraints*        ccs;
    UnsatisfiableConstraintInfos *    unsatisfiableX, *unsatisfiableY;
    NonOverlapConstraintsMode         avoidOverlaps;
    std::vector<straightener::Edge*>* straightenEdges;

    double bendWeight, potBendWeight;

    /*
     * determines whether we should leave some overlaps to be resolved
     * vertically when generating straightening constraints in the x-dim
     */
    bool xSkipping;

    /*
     * when using the gradient projection optimisation method, the following
     * controls whether the problem should be preconditioned by affine scaling
     */
    bool scaling;

    /*
     * if the Mosek quadratic programming environment is available it may be
     * used to solve each iteration of stress majorization... slow but useful
     * for testing
     */
    bool externalSolver;
    bool majorization;
};

}  // namespace cola
