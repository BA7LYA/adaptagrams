///
/// @file PosVertInf.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __POS_VERT_INF_HXX_26FC6787DF3B__
#define __POS_VERT_INF_HXX_26FC6787DF3B__

namespace avoid {

struct PosVertInf
{
    PosVertInf(double p, VertInf* vI, ScanVisDirFlags d = VisDirNone)
        : pos(p)
        , vert(vI)
        , dirs(d)
    {
    }

    bool operator<(const PosVertInf& rhs) const
    {
        if (pos != rhs.pos)
        {
            return pos < rhs.pos;
        }
        if ((vert->id == rhs.vert->id) && (vert->id == dummyOrthogID))
        {
            // Multiple dummy nodes can get placed at the same point for
            // multiple ShapeConnectionPins on junctions (outside of shapes).
            // We only need one at each position, so multiples can be seen
            // as equal here.
            return false;
        }
        if (vert->id != rhs.vert->id)
        {
            return vert->id < rhs.vert->id;
        }
        return dirs < rhs.dirs;
    }

    double   pos;
    VertInf* vert;

    // A bitfield marking the direction of visibility (in this dimension)
    // made up of VisDirDown (for visibility towards lower position values)
    // and VisDirUp (for visibility towards higher position values).
    //
    ScanVisDirFlags dirs;
};

}  // namespace avoid

#endif  // __POS_VERT_INF_HXX_26FC6787DF3B__
