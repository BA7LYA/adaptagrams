///
/// @file PositionStats.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/PositionStats.hxx"

namespace avoid {

void PositionStats::addVariable(Variable* v)
{
    double ai  = scale / v->scale;
    double bi  = v->offset / v->scale;
    double wi  = v->weight;
    AB        += wi * ai * bi;
    AD        += wi * ai * v->desiredPosition;
    A2        += wi * ai * ai;

    /*
    #ifdef LIBVPSC_LOGGING
        ofstream f(LOGFILE,ios::app);
        f << "adding v[" << v->id << "], blockscale=" << scale << ", despos="
          << v->desiredPosition << ", ai=" << ai << ", bi=" << bi
          << ", AB=" << AB << ", AD=" << AD << ", A2=" << A2;
    #endif
    */
}

}  // namespace avoid
