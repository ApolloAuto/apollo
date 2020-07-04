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

