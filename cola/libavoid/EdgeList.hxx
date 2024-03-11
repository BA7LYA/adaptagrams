///
/// @file EdgeList.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace avoid {

class EdgeList
{
public:
    friend class EdgeInf;
    EdgeList(bool orthogonal = false);
    ~EdgeList();
    void     clear(void);
    EdgeInf* begin(void);
    EdgeInf* end(void);
    int      size(void) const;

private:
    void addEdge(EdgeInf* edge);
    void removeEdge(EdgeInf* edge);

    bool         m_orthogonal;
    EdgeInf*     m_first_edge;
    EdgeInf*     m_last_edge;
    unsigned int m_count;
};

}  // namespace avoid
