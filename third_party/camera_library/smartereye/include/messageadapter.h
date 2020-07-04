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
#ifndef MESSAGEADAPTER_H
#define MESSAGEADAPTER_H

#include <QObject>
#include <QSet>
#include "satpext_global.h"
#include "service.h"
#include "blockhandler.h"

namespace SATP {
    class Protocol;
}

class SATPEXTSHARED_EXPORT MessageAdapter : public QObject, public Service, public SATP::BlockHandler
{
    Q_OBJECT
public:
    explicit MessageAdapter(SATP::Protocol *protocol, QObject *parent = nullptr);
    virtual ~MessageAdapter();
    //override BlockHandler
    bool handleReceiveBlock(uint32_t dataType, const char *block, int size);
    void handleReady();
    void registerUrgentMessage(int type);

protected:
    void handleMessage(int type, const char *message, int size);

private:
    SATP::Protocol *mProtocol;
    QSet<int> mUrgentMessages;
};

#endif // MESSAGEADAPTER_H
