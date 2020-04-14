#ifndef FRAMEEXT_H
#define FRAMEEXT_H

#if defined(Q_CC_MSVC)
#pragma warning(disable: 4200)
#endif

#include <cstdint>

#pragma pack(push, 1)

struct FrameDataExtHead
{
    uint32_t dataType;
    uint32_t dataSize;
    char data[0];

    enum{
        LaneExtData,
        ObstacleData,
        DrivingAreaData,
        OV491_EMBEDED_LINE,
    };
};

#pragma pack(pop)


#endif // FRAMEEXT_H

