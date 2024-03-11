///
/// @file ConstrainedFDLayout.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include <string>
#include <valarray>
#include <vector>

namespace vpsc {
class Rectangles;
}  // namespace vpsc

namespace cola {

class Edge;
class EdgeLengths;
class PreIteration;
class TestConvergence;
class TopologyAddonInterface;
class CompoundConstraints;
class ListOfNodeIndexes;
class DesiredPositions;
class RootCluster;
class UnsatisfiableConstraintInfos;

using Position = std::valarray<double>;

/**
 * @brief Implements a constrained force-directed layout algorithm.
 *
 * This method is based on a non-linear gradient projection technique.
 * Conceptually it's similar to a force directed method like
 * Fruchterman-Reingold---but using a more principled goal function and
 * optimisation techniques.
 */
class ConstrainedFDLayout
{
public:
    /**
     * @brief Constructs a constrained force-directed layout instance.
     *
     * If an empty edges (es) vector is passed to the constructor, then
     * constraint satisfaction will be performed, but no force-directed
     * layout.  In this case, idealLength and eLengths have no effect.
     *
     * Conversely, if no constraints or clusters are specified and no overlap
     * prevention is enabled, but edge info is given, then pure force-directed
     * layout will be performed.
     *
     * @param[in] rs  Bounding boxes of nodes at their initial positions.
     * @param[in] es  Simple pair edges, giving indices of the start and end
     *                nodes in rs.
     * @param[in] idealLength  A scalar modifier of ideal edge lengths in
     *                         eLengths or of 1 if no ideal lengths are
     *                         specified.
     * @param[in] eLengths  Individual ideal lengths for edges.
     *                      The actual ideal length used for the ith edge is
     *                      idealLength*eLengths[i], or if eLengths is nullptr a
     *                      then just idealLength is used (i.e., eLengths[i]
     *                      is assumed to be 1).
     * @param[in] done  A test of convergence operation called at the end of
     *                  each iteration (optional).  If not given, uses a
     *                  default TestConvergence object.
     * @param[in] preIteration  An operation called before each iteration
     *                          (optional).
     */
    ConstrainedFDLayout(
        const vpsc::Rectangles&        rs,
        const std::vector<cola::Edge>& es,
        const double                   idealLength,
        const EdgeLengths&             eLengths     = StandardEdgeLengths,
        TestConvergence*               doneTest     = nullptr,
        PreIteration*                  preIteration = nullptr
    );
    ~ConstrainedFDLayout();

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

    /**
     * @brief  Specify a set of compound constraints to apply to the layout.
     *
     * @param[in] ccs  The compound constraints.
     */
    void setConstraints(const cola::CompoundConstraints& ccs);

    /**
     * @brief  Specifies whether non-overlap constraints should be
     *         automatically generated between all nodes, as well as any
     *         exemptions to this.
     *
     * The optional second parameter indicates groups of nodes that should be
     * exempt from having non-overlap constraints generated between each other.
     * For example, you might want to do this for nodes representing ports, or
     * the child nodes in a particular cluster.
     *
     * @param[in] avoidOverlaps     New boolean value for this option.
     * @param[in] listOfNodeGroups  A list of groups of node indexes which will
     *                              not have non-overlap constraints generated
     *                              between each other.
     */
    void setAvoidNodeOverlaps(
        bool              avoidOverlaps,
        ListOfNodeIndexes listOfNodeGroups = ListOfNodeIndexes()
    );

    /**
     *  @brief  Set an addon for doing topology preserving layout.
     *
     *  It is expected that you would use the topology::ColaTopologyAddon()
     *  from libtopology rather than write your own.  This is done so that
     *  libcola does not have to depend on libtopology.
     *
     *  @param[in] topology  Instance of a class implementing the
     *                       TopologyAddonInterface.
     *  @sa topology::ColaTopologyAddon
     */
    void                    setTopology(TopologyAddonInterface* topology);
    TopologyAddonInterface* getTopology(void);

    void setDesiredPositions(DesiredPositions* desiredPositions);

    /**
     * @brief  Specifies an optional hierarchy for clustering nodes.
     *
     * @param[in] hierarchy  A pointer to a RootCluster object defining the
     *                      the cluster hierarchy (optional).
     */
    void setClusterHierarchy(RootCluster* hierarchy)
    {
        clusterHierarchy = hierarchy;
    }

    /**
     * @brief Register to receive information about unsatisfiable constraints.
     *
     * In the case of unsatisifiable constraints, the solver will drop
     * unsatisfiable constraints of lowest priority.  Information about these
     * will be written to these lists after each iteration of constrained
     * layout.
     *
     * param[out] unsatisfiableX A pointer to a UnsatisfiableConstraintInfos
     *                           object that will be used to record
     *                           unsatisfiable constraints in the x-dimension.
     * param[out] unsatisfiableY A pointer to a UnsatisfiableConstraintInfos
     *                           object that will be used to record
     *                           unsatisfiable constraints in the y-dimension.
     */
    void setUnsatisfiableConstraintInfo(
        UnsatisfiableConstraintInfos* unsatisfiableX,
        UnsatisfiableConstraintInfos* unsatisfiableY
    )
    {
        unsatisfiable.resize(2);
        unsatisfiable[0] = unsatisfiableX;
        unsatisfiable[1] = unsatisfiableY;
    }

    /**
     * @brief Finds a feasible starting position for nodes that satisfies the
     *        given constraints.
     *
     * Starts with an initial position (x, y) for the nodes.  This position
     * is then iteratively updated with a greedy heuristic that tries adding
     * additional constraints based on compound constraint priority to the
     * satisfiable set, so as to satisfy as many of the placement constraints
     * as possible.  This includes automatically generated constraints for
     * non-overlap and cluster containment.
     *
     * @param[in] xBorder  Optional border width to add to left and right
     *                     sides of rectangles. Defaults to 1.
     * @param[in] yBorder  Optional border width to add to top and bottom
     *                     sides of rectangles. Defaults to 1.
     *
     * @note This method doesn't do force-directed layout.  All forces are
     *       ignored and it merely satisfies the constraints with minimal
     *       movement to nodes.
     */
    void makeFeasible(double xBorder = 1, double yBorder = 1);

    /**
     * @brief  A convenience method that can be called from Java to free
     *         the memory of nodes (Rectangles), CompoundConstraints, etc.
     *
     * This assumes that the ConstrainedFDLayout instance takes ownership
     * of all the objects passed to it.
     *
     * This is useful because in SWIG we have problems with Java wrapper
     * classes going out of scope and causing objects like Rectanges to
     * sometimes be freed when the layout instance still needs them.  For
     * this reason we prevent the Java wrappers from deleting the internal
     * C++ instances, and let them be cleaned up later via this method.
     */
    void freeAssociatedObjects(void);

    //! @brief  Generates an SVG file containing debug output and code that
    //!         can be used to regenerate the instance.
    //!
    //! This method can be called before or after run() or makeFeasible()
    //! have been called.
    //!
    //! @param[in] filename  A string indicating the filename (without
    //!                      extension) for the output file.  Defaults to
    //!                      "libcola-debug.svg" if no filename is given.
    //!
    void outputInstanceToSVG(std::string filename = std::string());

    /**
     * @brief  Specifies whether neighbour stress should be used.
     *
     * Under neighbour stress, only the terms representing neighbouring
     * nodes contribute to the stress function. This can help to distribute
     * nodes more evenly, eliminating long-range forces.
     *
     * Default value is false.
     *
     * @param[in] useNeighbourStress  New boolean value for this option.
     */
    void setUseNeighbourStress(bool useNeighbourStress);

    /**
     * @brief  Retrieve a copy of the "D matrix" computed by the
     * computePathLengths method, linearised as a vector.
     *
     * This is especially useful for projects in SWIG target languages that want
     * to do their own computations with stress.
     *
     * D is the required euclidean distances between pairs of nodes
     * based on the shortest paths between them (using
     * m_idealEdgeLength*eLengths[edge] as the edge length, if eLengths array
     * is provided otherwise just m_idealEdgeLength).
     *
     * @return  vector representing the D matrix.
     */
    std::vector<double> readLinearD(void);

    /**
     * @brief  Retrieve a copy of the "G matrix" computed by the
     * computePathLengths method, linearised as a vector.
     *
     * * This is especially useful for projects in SWIG target languages that
     * want to do their own computations with stress.
     *
     * G is a matrix of unsigned ints such that G[u][v]=
     *   0 if there are no forces required between u and v
     *     (for example, if u and v are in unconnected components)
     *   1 if attractive forces are required between u and v
     *     (i.e. if u and v are immediately connected by an edge and there is
     *      no topology route between u and v (for which an attractive force
     *      is computed elsewhere))
     *   2 if no attractive force is required between u and v but there is
     *     a connected path between them.
     *
     * @return  vector representing the G matrix.
     */
    std::vector<unsigned> readLinearG(void);

    double computeStress() const;

private:
    unsigned              n;  // number of nodes
    std::valarray<double> X, Y;
    vpsc::Rectangles      boundingBoxes;

    double applyForcesAndConstraints(
        const vpsc::Dim dim,
        const double    oldStress
    );
    double computeStepSize(
        const SparseMatrix&          H,
        const std::valarray<double>& g,
        const std::valarray<double>& d
    ) const;
    void computeDescentVectorOnBothAxes(
        const bool             xaxis,
        const bool             yaxis,
        double                 stress,
        std::valarray<double>& x0,
        std::valarray<double>& x1
    );
    void   moveTo(const vpsc::Dim dim, std::valarray<double>& target);
    double applyDescentVector(
        const std::valarray<double>& d,
        const std::valarray<double>& oldCoords,
        std::valarray<double>&       coords,
        const double                 oldStress,
        double                       stepsize
        /*,topology::TopologyConstraints *s=nullptr*/
    );
    void computePathLengths(
        const std::vector<Edge>& es,
        std::valarray<double>    eLengths
    );
    void generateNonOverlapAndClusterCompoundConstraints(vpsc::Variables (&vs
    )[2]);
    void handleResizes(const Resizes&);
    void setPosition(std::valarray<double>& pos);
    void moveBoundingBoxes();
    bool noForces(double, double, unsigned) const;
    void computeForces(
        const vpsc::Dim        dim,
        SparseMap&             H,
        std::valarray<double>& g
    );
    void recGenerateClusterVariablesAndConstraints(
        vpsc::Variables (&vars)[2],
        unsigned int&                priority,
        cola::NonOverlapConstraints* noc,
        Cluster*                     cluster,
        cola::CompoundConstraints&   idleConstraints
    );
    std::vector<double> offsetDir(double minD);

    void computeNeighbours(std::vector<Edge> es);

    std::vector<std::vector<unsigned>> neighbours;
    std::vector<std::vector<double>>   neighbourLengths;
    TestConvergence*                   done;
    bool using_default_done;  // Whether we allocated a default TestConvergence
                              // object.
    PreIteration*             preIteration;
    cola::CompoundConstraints ccs;
    double**                  D;
    unsigned short**          G;
    double                    minD;
    PseudoRandom              random;

    TopologyAddonInterface*                    topologyAddon;
    std::vector<UnsatisfiableConstraintInfos*> unsatisfiable;
    bool                                       rungekutta;
    DesiredPositions*                          desiredPositions;
    cola::CompoundConstraints                  extraConstraints;

    RootCluster*                clusterHierarchy;
    double                      rectClusterBuffer;
    double                      m_idealEdgeLength;
    bool                        m_generateNonOverlapConstraints;
    bool                        m_useNeighbourStress;
    const std::valarray<double> m_edge_lengths;

    NonOverlapConstraintExemptions* m_nonoverlap_exemptions;

    friend class topology::ColaTopologyAddon;
    friend class dialect::Graph;
};

}  // namespace cola
