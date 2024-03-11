///
/// @file VertInfList.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __VERT_INF_LIST_HXX_AEA7F8532A31__
#define __VERT_INF_LIST_HXX_AEA7F8532A31__

namespace avoid {

class Point;
class VertID;
class VertInf;

// A linked list of all the vertices in the router instance.  All the
// connector endpoints are listed first, then all the shape vertices.
// Dummy vertices inserted for orthogonal routing are classed as shape
// vertices but have VertID(0, 0).
//
class VertInfList
{
public:
    VertInfList();
    void         addVertex(VertInf* vert);
    VertInf*     removeVertex(VertInf* vert);
    VertInf*     getVertexByID(const VertID& id);
    VertInf*     getVertexByPos(const Point& p);
    VertInf*     shapesBegin(void);
    VertInf*     connsBegin(void);
    VertInf*     end(void);
    unsigned int connsSize(void) const;
    unsigned int shapesSize(void) const;

private:
    VertInf*     _firstShapeVert;
    VertInf*     _firstConnVert;
    VertInf*     _lastShapeVert;
    VertInf*     _lastConnVert;
    unsigned int _shapeVertices;
    unsigned int _connVertices;
};

}  // namespace avoid

#endif  // __VERT_INF_LIST_HXX_AEA7F8532A31__
