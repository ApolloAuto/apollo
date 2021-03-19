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
#ifndef RTDBADAPTER_H
#define RTDBADAPTER_H

#include <QObject>
#include "satpext_global.h"
#include "rtdbservice.h"
#include "blockhandler.h"

namespace SATP {
    class Protocol;
}

class SATPEXTSHARED_EXPORT RtdbSender : public RtdbService, public SATP::BlockHandler
{
    Q_OBJECT
public:
    RtdbSender(RealtimeDatabase *rtdb, SATP::Protocol *protocol, QObject *parent = nullptr);
    virtual ~RtdbSender();
    //override BlockHandler
    bool handleReceiveBlock(uint32_t dataType, const char *block, int size);
    void handleReady();

protected:
    void handleMessage(int type, const char *message, int size);
    void sendRtdb();

private:
    SATP::Protocol *mProtocol;
};

#endif // RTDBADAPTER_H
