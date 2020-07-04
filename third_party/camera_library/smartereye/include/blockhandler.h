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
#ifndef BLOCKHANDLER_H
#define BLOCKHANDLER_H

#include <cstdint>
#include <memory>

using namespace std;
namespace SATP {

class BlockHandler
{
public:
    virtual bool handleReceiveBlock(uint32_t dataType, const char *block, int size) = 0;
    virtual void handleReset(){}
    virtual void handleReady(){}
};

class EnhancedHandler : public BlockHandler
{
public:
    EnhancedHandler() {}
    virtual bool handleReceiveBlockEnhanced(uint32_t dataType, shared_ptr<const char> &block, int size){
        handleReceiveBlock(dataType, block.get(), size);
    }
};

}
#endif // BLOCKHANDLER_H
