#ifndef DATAUNIT_H
#define DATAUNIT_H
#include "protocolunit.h"

#pragma pack(push, 1)

namespace SATP {

struct DataUnitHead
{
    uint32_t dataType;
    uint16_t continued;

    inline DataUnitHead(uint32_t type = 0):
        dataType(type),
        continued(0)
    {
    }

    inline const char *data()
    {
        return (const char*)this;
    }

    inline const char *body()
    {
        return (const char*)this + sizeof(DataUnitHead);
    }

    inline static int bodySize(int dataUnitSize)
    {
        return dataUnitSize - sizeof(DataUnitHead);
    }
};

inline int maxBlockSize()
{
    return maxDataUnitSize() - sizeof(DataUnitHead);
}

}

#pragma pack(pop)
#endif // DATAUNIT_H
