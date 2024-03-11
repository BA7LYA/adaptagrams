///
/// @file PreIteration.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace cola {

class Locks;
class Resizes;

/**
 * @brief A default functor that is called before each iteration in the
 *        main loop of the ConstrainedFDLayout::run() method.
 *
 * Override the operator() for things like locking the position of nodes
 * for the duration of the iteration.
 *
 * If the operator() returns false the subsequent iterations are
 * abandoned, i.e., layout ends immediately.  You can make it return true
 * when a user-interrupt is detected, for instance.
 */
class PreIteration
{
public:
    /**
     * @brief  Constructs a PreIteration object that handles locking and
     *         resizing of nodes.
     *
     * @param[in] locks    A list of nodes (by index) and positions at which
     *                     they should be locked.
     * @param[in] resizes  A list of nodes (by index) and required dimensions
     *                     for their bounding rects to be resized to.
     */
    PreIteration(
        Locks&   locks   = __locksNotUsed,
        Resizes& resizes = __resizesNotUsed
    )
        : locks(locks)
        , resizes(resizes)
        , changed(true)
    {
    }

    PreIteration(Resizes& resizes)
        : locks(__locksNotUsed)
        , resizes(resizes)
        , changed(true)
    {
    }

// To prevent C++ objects from being destroyed in garbage collected languages
// when the libraries are called from SWIG, we hide the declarations of the
// destructors and prevent generation of default destructors.
#ifndef SWIG
    virtual ~PreIteration() {}
#endif

    virtual bool operator()()
    {
        return true;
    }

    Locks&   locks;
    Resizes& resizes;
    bool     changed;

private:
    static Locks   __locksNotUsed;
    static Resizes __resizesNotUsed;
};

}  // namespace cola
