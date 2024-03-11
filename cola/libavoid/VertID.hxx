///
/// @file VertID.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __VERT_ID_HXX_CDA879B48AA6__
#define __VERT_ID_HXX_CDA879B48AA6__

namespace avoid {

class VertID
{
public:
    unsigned int   objID;
    unsigned short vn;
    // Properties:
    VertIDProps    props;

    static const unsigned short src;
    static const unsigned short tar;

    static const VertIDProps PROP_ConnPoint;
    static const VertIDProps PROP_OrthShapeEdge;
    static const VertIDProps PROP_ConnectionPin;
    static const VertIDProps PROP_ConnCheckpoint;
    static const VertIDProps PROP_DummyPinHelper;

    VertID();
    VertID(unsigned int id, unsigned short n, VertIDProps p = 0);
    VertID(const VertID& other);
    VertID& operator=(const VertID& rhs);
    VertID  operator+(const int& rhs) const;
    VertID  operator-(const int& rhs) const;
    VertID& operator++(int);
    bool    operator==(const VertID& rhs) const;
    bool    operator!=(const VertID& rhs) const;
    bool    operator<(const VertID& rhs) const;

    void                 print(FILE* file = stdout) const;
    void                 db_print(void) const;
    friend std::ostream& operator<<(std::ostream& os, const VertID& vID);

    // Property tests:
    inline bool isOrthShapeEdge(void) const
    {
        return (props & PROP_OrthShapeEdge) ? true : false;
    }

    inline bool isConnPt(void) const
    {
        return (props & PROP_ConnPoint) ? true : false;
    }

    inline bool isConnectionPin(void) const
    {
        return (props & PROP_ConnectionPin) ? true : false;
    }

    inline bool isConnCheckpoint(void) const
    {
        return (props & PROP_ConnCheckpoint) ? true : false;
    }

    inline bool isDummyPinHelper(void) const
    {
        return (props & PROP_DummyPinHelper) ? true : false;
    }
};

using VertIDProps = unsigned short;

}  // namespace avoid

#endif  // __VERT_ID_HXX_CDA879B48AA6__
