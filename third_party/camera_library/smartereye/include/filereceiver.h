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
#ifndef FILERECEIVER_H
#define FILERECEIVER_H

#include <QString>
#include <QVector>
#include "satpext_global.h"
#include "blockhandler.h"

class QFile;

namespace SATP {

class Protocol;
class FileReceiverHandler;

class SATPEXTSHARED_EXPORT FileReceiver : public BlockHandler
{
public:
    explicit FileReceiver(Protocol *protocol);
    virtual ~FileReceiver();
    FileReceiver(const FileReceiver&&){}
    //override.
    bool handleReceiveBlock(uint32_t dataType, const char *block, int size);

    //FileReceiver
    void registerReceiverHandler(FileReceiverHandler *receiverHandler);
    double getReceiveProgress();
    void setRecvFileDir(const QString &dir);

protected:
    void handleFileHeader(const char *block);
    void handleFileTail(const char *block);
    void handleFileData(const char *block, int size);
    void sendFileResp(bool finished = false);
    void raiseReceiverHandlers(const QString &filePath);

private:
    QVector<FileReceiverHandler *> mReceiverHandlers;
    Protocol     *mProtocol;

    int           mFileSize;
    QFile        *mFile;
    int           mReceived;
    int           mPacketCount;
    QString       mRecvFileDir;
};

}
#endif // FILERECEIVER_H
