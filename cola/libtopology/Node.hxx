///
/// @file Node.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-11
/// @copyright Copyright (c) 2024
///

#include <vector>

namespace topoloty {

/**
 * @brief  Topology representation for a node.
 *
 * Each node is associated with a rectangle and solver variables
 * for the x and y axes.
 *
 * @note You shouldn't need to create these yourself, but you may
 *       extract them from an existing ColaTopologyAddon and construct
 *       a new ColaTopologyAddon with that same topology information.
 */
class Node
{
public:
    // the index of the associated node / variable / rectangle
    const unsigned id;

    // the bounding box of the associated node
    vpsc::Rectangle* rect;

    /*
     * When an edge path is being defined externally with a vector of
     * EdgePoint, a variable would not be specified.
     * @param id
     * @param r
     */
    Node(unsigned id, vpsc::Rectangle* r, vpsc::Variable* v = nullptr);

    void   setDesiredPos(double d, double weight = 1.0);
    double initialPos(vpsc::Dim scanDim) const;
    double finalPos() const;
    double posOnLine(vpsc::Dim scanDim, double alpha) const;

    vpsc::Variable* getVar() const
    {
        return var;
    }

    // variable positions used by solver
    vpsc::Variable* var;
};

/**
 * @brief  A vector of pointers to Node objects.
 */
using Nodes = std::vector<Node*>;

}  // namespace topoloty
