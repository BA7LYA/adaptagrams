///
/// @file TopologyAddonInterface.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace avoid {

// NOTE: This is an internal helper class that should not be used by the user.
//
// It is used by libtopology to add additional functionality to libavoid
// while keeping libavoid dependency free.
class TopologyAddonInterface
{
public:
    TopologyAddonInterface() {}

    virtual ~TopologyAddonInterface() {}

    virtual TopologyAddonInterface* clone(void) const
    {
        return new TopologyAddonInterface(*this);
    }

    virtual void improveOrthogonalTopology(Router* router)
    {
        (void)(router);  // Avoid unused parameter warning.
    }

    virtual bool outputCode(FILE* fp) const
    {
        (void)(fp);  // Avoid unused parameter warning.
        return false;
    }

    virtual bool outputDeletionCode(FILE* fp) const
    {
        (void)(fp);  // Avoid unused parameter warning.
        return false;
    }
};

}  // namespace avoid
