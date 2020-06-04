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
#ifndef SATPKEYS
#define SATPKEYS

#include "dataunit.h"

#ifdef _WIN64
#pragma warning(disable: 4200)
#endif

#pragma pack(push, 1)

struct DataUnitTypeExt
{
    enum Enumation
    {
        FileHeader = 0,
        FileTail = 1,
        FileData = 2,
        FileResp = 3,
        Message = 64,
        Rtdb = 65,
        Image = 66,
        UpdateFirmware = 67,
        RequestRtdb = 68,
        RequestImage = 69,
        ResponseUpdate = 70,
        RawImageFrame = 71,
        MetaData = 72,
        RequestMetaData = 73,
        EnableMaxSendFrameInterval = 74,
        ChangeRtdbItem = 75,
    };
};

struct SaptPort
{
    enum Enumation
    {
        Default = 52404,              //quint16
        DefaultUdp = 52410,
    };
};

union UniqueKey
{
    uint32_t longKey;
    struct
    {
        uint16_t lowWord;
        uint16_t highWord;
    }doubleKey;

    UniqueKey(uint32_t aLongkey)
    {
        longKey = aLongkey;
    }

    UniqueKey(uint16_t lowWord, uint16_t highWord)
    {
        doubleKey.lowWord = lowWord;
        doubleKey.highWord = highWord;
    }

    uint16_t lowWord()
    {
        return doubleKey.lowWord;
    }

    uint16_t highWord()
    {
        return doubleKey.highWord;
    }
private:
    UniqueKey();
};


struct BlockFileHeader
{
    uint32_t fileSize;
    inline const char *fileName() const
    {
        return reinterpret_cast<const char*>(this) + sizeof(BlockFileHeader);
    }
};

struct BlockFileResp
{
    uint32_t received;
    uint16_t continued;
    inline const char *fileName()
    {
        return (const char*)this + sizeof(BlockFileResp);
    }
};

struct RawImageFrame
{
    uint16_t frameId;
    int64_t  time;
    uint16_t index;
    uint16_t format;
    uint16_t width;
    uint16_t height;
    uint32_t speed;
    uint32_t dataSize;
    uint8_t  image[0];
};

#pragma pack(pop)

#endif // SATPKEYS

