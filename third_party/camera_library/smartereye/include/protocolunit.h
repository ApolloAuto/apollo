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
