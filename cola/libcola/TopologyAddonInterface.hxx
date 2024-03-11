///
/// @file TopologyAddonInterface.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace cola {

/**
 * @brief  Interface for writing COLA addons to handle topology preserving
 *         layout.
 */
class TopologyAddonInterface
{
public:
    TopologyAddonInterface() {}

    virtual ~TopologyAddonInterface() {}

    virtual TopologyAddonInterface* clone(void) const
    {
        return new TopologyAddonInterface(*this);
    }

    virtual void freeAssociatedObjects(void) {}

    virtual void handleResizes(
        const Resizes&             resizeList,
        unsigned                   n,
        std::valarray<double>&     X,
        std::valarray<double>&     Y,
        cola::CompoundConstraints& ccs,
        vpsc::Rectangles&          boundingBoxes,
        cola::RootCluster*         clusterHierarchy
    )
    {
        COLA_UNUSED(resizeList);
        COLA_UNUSED(n);
        COLA_UNUSED(X);
        COLA_UNUSED(Y);
        COLA_UNUSED(ccs);
        COLA_UNUSED(boundingBoxes);
        COLA_UNUSED(clusterHierarchy);
    }

    virtual void computePathLengths(unsigned short** G)
    {
        COLA_UNUSED(G);
    }

    virtual double computeStress(void) const
    {
        return 0;
    }

    virtual bool useTopologySolver(void) const
    {
        return false;
    }

    virtual void makeFeasible(
        bool               generateNonOverlapConstraints,
        vpsc::Rectangles&  boundingBoxes,
        cola::RootCluster* clusterHierarchy
    )
    {
        COLA_UNUSED(generateNonOverlapConstraints);
        COLA_UNUSED(boundingBoxes);
        COLA_UNUSED(clusterHierarchy);
    }

    virtual void moveTo(
        const vpsc::Dim        dim,
        vpsc::Variables&       vs,
        vpsc::Constraints&     cs,
        std::valarray<double>& coords,
        cola::RootCluster*     clusterHierarchy
    )
    {
        COLA_UNUSED(dim);
        COLA_UNUSED(vs);
        COLA_UNUSED(cs);
        COLA_UNUSED(coords);
        COLA_UNUSED(clusterHierarchy);
    }

    virtual double applyForcesAndConstraints(
        ConstrainedFDLayout*   layout,
        const vpsc::Dim        dim,
        std::valarray<double>& g,
        vpsc::Variables&       vs,
        vpsc::Constraints&     cs,
        std::valarray<double>& coords,
        DesiredPositionsInDim& des,
        double                 oldStress
    )
    {
        COLA_UNUSED(layout);
        COLA_UNUSED(dim);
        COLA_UNUSED(g);
        COLA_UNUSED(vs);
        COLA_UNUSED(cs);
        COLA_UNUSED(coords);
        COLA_UNUSED(des);
        COLA_UNUSED(oldStress);
        return 0;
    }
};

}  // namespace cola
