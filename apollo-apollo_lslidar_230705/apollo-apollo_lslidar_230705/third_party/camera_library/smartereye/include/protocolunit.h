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
#ifndef PROTOCOLUNIT_H
#define PROTOCOLUNIT_H

#include <cstdint>

#pragma pack(push, 1)

#define SATP_MAX_PROTOCOL_UNIT_LEN 500000

namespace SATP {

enum ProtocolUnitFormat
{
    FixedProtocolUnitFormat,
    LongProtocolUnitFormat
};

//only apply for Fixed format;
enum ProtocolUnitType
{
    HeartBeatReq,
    HeartBeatResp,
    HeartBeatCon
};

struct ProtocolUnitHead
{
    uint16_t token;
    uint32_t dataUnitSize;
    uint16_t format;
    uint16_t type;
};

inline int maxDataUnitSize()
{
    return SATP_MAX_PROTOCOL_UNIT_LEN - sizeof(ProtocolUnitHead);
}

}

#pragma pack(pop)

#endif // PROTOCOLUNIT_H
