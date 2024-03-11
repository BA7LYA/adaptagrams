///
/// @file ConnRerouteFlagDelegate.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include <list>

namespace avoid {

class ConnRef;

// NOTE: This is an internal helper class that should not be used by the user.
//
// This class allows edges in the visibility graph to store a
// pointer to a boolean registering when a connector needs to
// reroute, while allowing connectors to be deleted without
// needing to scan and remove these references from the visibility
// graph.  Instead the bool is stored in this delegate and the
// connector is alerted later, so long as it hasn't since been
// deleted.
class ConnRerouteFlagDelegate
{
public:
    ConnRerouteFlagDelegate();
    ~ConnRerouteFlagDelegate();
    bool* addConn(ConnRef* conn);
    void  removeConn(ConnRef* conn);
    void  alertConns(void);

private:
    std::list<std::pair<ConnRef*, bool>> m_mapping;
};

}  // namespace avoid
