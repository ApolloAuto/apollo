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
#ifndef FILESENDER_H
#define FILESENDER_H

#include "blockhandler.h"
#include "satpext_global.h"
#include <QRunnable>
#include <QString>

class QFile;

namespace SATP {

class Protocol;
class FileSenderHandler;

class SATPEXTSHARED_EXPORT FileSender : public BlockHandler, public QRunnable
{
public:
    FileSender(Protocol *protocol, FileSenderHandler *senderHandler);
    virtual ~FileSender();
    void run();
    void send(const QString &filePath);
    //override.
    bool handleReceiveBlock(uint32_t dataType, const char *block, int size);
    void handleReset();
    double getSendProgress();

protected:
    void sendFileSynchronously();
    void sendFileHeader(int fileSize);
    void sendFileTail();
    void handleFileResp(const char *block);

private:
    Protocol *mProtocol;
    FileSenderHandler  *mSenderHandler;

    QString mFilePath;
    QString mFileName;
    qint64 mFileSize;
    qint64 mSizeToSend;
    bool mIsRunning;
    bool mIsCanceling;
};

}
#endif // FILESENDER_H
