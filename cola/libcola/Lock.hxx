///
/// @file Lock.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include <vector>

namespace cola {

/**
 * @brief A Lock specifies a required position for a node.
 */
class Lock
{
public:
    Lock() {}

    /**
     * @brief  Constructs a Lock object for a given node and position.
     *
     * @param[in] id  The index of the node in the Rectangles vector.
     * @param[in] X   The node's fixed position in the x-dimension.
     * @param[in] Y   The node's fixed position in the y-dimension.
     */
    Lock(unsigned id, double X, double Y)
        : id(id)
        , x(X)
        , y(Y)
    {
    }

    unsigned getID() const
    {
        return id;
    }

    double pos(vpsc::Dim dim) const
    {
        return dim == vpsc::HORIZONTAL ? x : y;
    }

private:
    unsigned id;
    double   x;
    double   y;
};

//! @brief A vector of Lock objects.
using Locks = std::vector<cola::Lock>;

}  // namespace cola
