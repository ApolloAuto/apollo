/******************************************************************************
 * Copyright 2020 The Beijing Smarter Eye Technology Co.Ltd Authors. All
 * Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
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
