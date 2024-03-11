///
/// @file ImproveOrthogonalRoutes.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __IMPROVE_ORTHOGONAL_ROUTES_HXX_996AE8C8F826__
#define __IMPROVE_ORTHOGONAL_ROUTES_HXX_996AE8C8F826__

namespace avoid {

class ImproveOrthogonalRoutes
{
public:
    ImproveOrthogonalRoutes(Router* router);
    void execute(void);

private:
    void simplifyOrthogonalRoutes(void);
    void buildOrthogonalNudgingOrderInfo(void);
    void nudgeOrthogonalRoutes(size_t dimension, bool justUnifying = false);

    Router*          m_router;
    PtOrderMap       m_point_orders;
    UnsignedPairSet  m_shared_path_connectors_with_common_endpoints;
    ShiftSegmentList m_segment_list;
};

}  // namespace avoid

#endif  // __IMPROVE_ORTHOGONAL_ROUTES_HXX_996AE8C8F826__
