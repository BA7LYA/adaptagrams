///
/// @file Resize.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace cola {

/**
 * @brief A Resize specifies a new required bounding box for a node.
 */
class Resize
{
public:
    Resize() {}

    /**
     * @brief  Constructs a Resize object for a given node and it's new
     *         bounding box.
     *
     * @param[in] id  The index of the node in the Rectangles vector.
     * @param[in] x   The minimum horizontal value for the node's new
     *                bounding box.
     * @param[in] y   The minimum vertical value for the node's new
     *                bounding box.
     * @param[in] w   The width value for the node's new bounding box.
     * @param[in] h   The height value for the node's new bounding box.
     */
    Resize(unsigned id, double x, double y, double w, double h)
        : id(id)
        , target(x, x + w, y, y + h)
    {
    }

    unsigned getID() const
    {
        return id;
    }

    const vpsc::Rectangle* getTarget() const
    {
        return &target;
    }

private:
    unsigned        id;
    vpsc::Rectangle target;
};

//! @brief A vector of Resize objects.
using Resizes = std::vector<cola::Resize>;

}  // namespace cola
