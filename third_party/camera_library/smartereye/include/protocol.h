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
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>

namespace SATP {

class BlockHandler;

class Protocol
{
public:
    enum TramsmitPriority {
        DropWhenBusy = 0,
        EnqueueForcedly,
        WaitToSend
    };

    virtual bool isConnected() = 0;
    virtual bool isAppendable() = 0;
    virtual void sendBlock(uint32_t dataType, const char *block, int size, TramsmitPriority priority = DropWhenBusy) = 0;
    virtual void registerBlockHandler(BlockHandler *blockHandler) = 0;
    virtual void unregisterBlockHandler(BlockHandler *blockHandler) = 0;
};

}
#endif // PROTOCOL_H
