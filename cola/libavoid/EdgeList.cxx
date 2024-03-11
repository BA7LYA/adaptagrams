///
/// @file EdgeList.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/EdgeList.hxx"

#include "libavoid/EdgeInf.hxx"

namespace avoid {

EdgeList::EdgeList(bool orthogonal)
    : m_orthogonal(orthogonal)
    , m_first_edge(nullptr)
    , m_last_edge(nullptr)
    , m_count(0)
{
}

EdgeList::~EdgeList()
{
    clear();
}

void EdgeList::clear(void)
{
    while (m_first_edge)
    {
        // The Edge destructor results in EdgeList:::removeEdge() being called
        // for this edge and m_first_edge being updated to the subsequent edge
        // in the EdgeList.
        delete m_first_edge;
    }
    COLA_ASSERT(m_count == 0);
    m_last_edge = nullptr;
}

int EdgeList::size(void) const
{
    return m_count;
}

void EdgeList::addEdge(EdgeInf* edge)
{
    // Dummy connections for ShapeConnectionPins won't be orthogonal,
    // even in the orthogonal visibility graph.
    COLA_UNUSED(m_orthogonal);
    COLA_ASSERT(
        !m_orthogonal || edge->isOrthogonal() || edge->isDummyConnection()
    );

    if (m_first_edge == nullptr)
    {
        COLA_ASSERT(m_last_edge == nullptr);

        m_last_edge  = edge;
        m_first_edge = edge;

        edge->lstPrev = nullptr;
        edge->lstNext = nullptr;
    }
    else
    {
        COLA_ASSERT(m_last_edge != nullptr);

        m_last_edge->lstNext = edge;
        edge->lstPrev        = m_last_edge;

        m_last_edge = edge;

        edge->lstNext = nullptr;
    }
    m_count++;
}

void EdgeList::removeEdge(EdgeInf* edge)
{
    if (edge->lstPrev)
    {
        edge->lstPrev->lstNext = edge->lstNext;
    }
    if (edge->lstNext)
    {
        edge->lstNext->lstPrev = edge->lstPrev;
    }
    if (edge == m_last_edge)
    {
        m_last_edge = edge->lstPrev;
        if (edge == m_first_edge)
        {
            m_first_edge = nullptr;
        }
    }
    else if (edge == m_first_edge)
    {
        m_first_edge = edge->lstNext;
    }

    edge->lstPrev = nullptr;
    edge->lstNext = nullptr;

    m_count--;
}

EdgeInf* EdgeList::begin(void)
{
    return m_first_edge;
}

EdgeInf* EdgeList::end(void)
{
    return nullptr;
}

}  // namespace avoid
