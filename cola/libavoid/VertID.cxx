///
/// @file VertID.cxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#include "libavoid/VertID.hxx"

namespace avoid {

const unsigned short VertID::src = 1;
const unsigned short VertID::tar = 2;

// Property flags:
const unsigned short VertID::PROP_ConnPoint      = 1;
const unsigned short VertID::PROP_OrthShapeEdge  = 2;
const unsigned short VertID::PROP_ConnectionPin  = 4;
const unsigned short VertID::PROP_ConnCheckpoint = 8;
const unsigned short VertID::PROP_DummyPinHelper = 16;

VertID::VertID() {}

VertID::VertID(unsigned int id, unsigned short n, VertIDProps p)
    : objID(id)
    , vn(n)
    , props(p)
{
}

VertID::VertID(const VertID& other)
    : objID(other.objID)
    , vn(other.vn)
    , props(other.props)
{
}

VertID& VertID::operator=(const VertID& rhs)
{
    // Gracefully handle self assignment
    // if (this == &rhs) return *this;

    objID = rhs.objID;
    vn    = rhs.vn;
    props = rhs.props;

    return *this;
}

bool VertID::operator==(const VertID& rhs) const
{
    if ((objID != rhs.objID) || (vn != rhs.vn))
    {
        return false;
    }
    return true;
}

bool VertID::operator!=(const VertID& rhs) const
{
    if ((objID != rhs.objID) || (vn != rhs.vn))
    {
        return true;
    }
    return false;
}

bool VertID::operator<(const VertID& rhs) const
{
    if ((objID < rhs.objID) || ((objID == rhs.objID) && (vn < rhs.vn)))
    {
        return true;
    }
    return false;
}

VertID VertID::operator+(const int& rhs) const
{
    return VertID(objID, vn + rhs, props);
}

VertID VertID::operator-(const int& rhs) const
{
    return VertID(objID, vn - rhs, props);
}

VertID& VertID::operator++(int)
{
    vn += 1;
    return *this;
}

void VertID::print(FILE* file) const
{
    fprintf(file, "[%u,%d, p=%u]", objID, vn, (unsigned int)props);
}

void VertID::db_print(void) const
{
    db_printf("[%u,%d, p=%u]", objID, vn, (unsigned int)props);
}

}  // namespace avoid
