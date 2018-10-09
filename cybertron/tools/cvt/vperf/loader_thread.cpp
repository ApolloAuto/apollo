/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "loader_thread.h"
#include <QCoreApplication>
#include <QSemaphore>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include "has_data_come_event.h"
#include "perf_data_base.h"

namespace {
constexpr int BufferCount = 4;
constexpr int BufferSize = 1 << 20;

struct IOBuffer {
  int _size;
  char *_buffer;
};

struct ReaderWriterBuffer {
  // just for two thread: one reader , one writer

  IOBuffer _byteArray[BufferCount];

  QSemaphore _wSem;
  QSemaphore _rSem;
  int _wIndex;
  int _rIndex;

  ReaderWriterBuffer()
      : _byteArray(), _wSem(BufferCount), _rSem(0), _wIndex(0), _rIndex(0) {
    for (int i = 0; i < BufferCount; ++i) {
      _byteArray[i]._buffer = new char[BufferSize];
      memset(_byteArray[i]._buffer, 0, BufferSize);
    }
  }

  ~ReaderWriterBuffer() {
    for (int i = 0; i < BufferCount; ++i) {
      delete _byteArray[i]._buffer;
    }
  }

  IOBuffer *getWriterBuffer(void) {
    _wSem.acquire();
    IOBuffer *ret = &_byteArray[_wIndex++];
    _wIndex %= BufferCount;
    return ret;
  }

  IOBuffer *getReaderBuffer(void) {
    _rSem.acquire();
    IOBuffer *ret = &_byteArray[_rIndex++];
    _rIndex %= BufferCount;
    return ret;
  }

  void releaseWriterBuffer(void) { _wSem.release(); }

  void releaseReaderBuffer(void) { _rSem.release(); }

  bool canWrite(void) const { return _wSem.available(); }

  bool canRead(void) const { return _rSem.available(); }
};
}

LoaderThread::LoaderThread(QObject *parent)
    : QThread(parent), _canRead(false) {}

void LoaderThread::run() {
  ReaderWriterBuffer buffer;

  bool canWrite = true;
  _canRead = true;
  auto writerFunc = [&buffer, this, &canWrite]() {

    std::ifstream file(this->_dataFileName.toStdString());

    if (!file.is_open()) {
      std::cerr << "Cannot open fille (" << this->_dataFileName.toStdString()
                << ")" << std::endl;

      canWrite = false;
      buffer.releaseReaderBuffer();
      return;
    }

    while (canWrite && this->_canRead) {
      IOBuffer *ba = buffer.getWriterBuffer();
      if (!this->_canRead) break;

      file.read(ba->_buffer, BufferSize);
      if (file.eof()) {
        canWrite = false;
      }

      ba->_size = file.gcount();
      buffer.releaseReaderBuffer();
    }

    file.close();
  };

  std::thread writer(writerFunc);

  PerfDatabase *dataBase = PerfDatabase::instance();
  dataBase->reset();

  int index = 0;
  bool isFirst = true;
  while ((canWrite || buffer.canRead()) && _canRead) {
    PerfBlockDatabase *blockDatabase = dataBase->getSubDataBase(index);
    if (blockDatabase) {
      QStringList strList;
      QString line;
      auto readLine = [&strList, &line](const char *&dataPtr,
                                        const char *const endPtr) -> bool {
        bool foundNewLine = false;
        while (dataPtr < endPtr) {
          char ch = *dataPtr++;
          if (ch == '\n') {
            foundNewLine = true;
            break;
          } else if (ch == '\t') {
            strList.append(line);
            line.clear();
          } else {
            line.append(ch);
          }
        }

        return foundNewLine;
      };

      IOBuffer *ba = buffer.getReaderBuffer();

      const char *dataPtr = ba->_buffer;
      const char *const endPtr = dataPtr + ba->_size;

      if (isFirst) {
        if (readLine(dataPtr, endPtr)) {
          int64_t tmp = line.toLongLong();
          dataBase->setStartTimeStamp(tmp);
          isFirst = false;
          line.clear();
        }
      }

      while (dataPtr < endPtr) {
        if (readLine(dataPtr, endPtr)) {
          if (strList.count() > 0) {
            int eventType = strList.at(0).toInt();
            if (eventType == 0) {
              if (strList.count() == 7) {
                blockDatabase->addTimeBlockData(
                    TimeBlockData::Format::Format8, strList.at(3).toInt(),
                    strList.at(2).toStdString(), strList.at(1).toInt() - 1,
                    strList.at(5).toLongLong(), strList.at(6).toLongLong(),
                    line.toLongLong());
              }

              if (strList.count() == 10) {
                TimeBlockData *data = blockDatabase->addTimeBlockData(
                    TimeBlockData::Format::Format11, strList.at(3).toInt(),
                    strList.at(2).toStdString(), strList.at(1).toInt() - 1,
                    strList.at(5).toLongLong(), strList.at(6).toLongLong(),
                    strList.at(7).toLongLong());
                if (data) {
                  data->setPreEventId(strList.at(8).toInt());
                  data->setPreEndTime(strList.at(9).toLongLong());
                  data->setLatency(line.toInt());
                }
              }
            }
          }

          strList.clear();
          line.clear();
        }
      }

      HasDataComeEvent *e = new HasDataComeEvent(index++);
      QCoreApplication::postEvent(parent(), e);
    } else {
      _canRead = false;
      std::cerr << "cannot get subdatabase with index = " << index << std::endl;
    }

    buffer.releaseWriterBuffer();
  }  // end while

  writer.join();
}
