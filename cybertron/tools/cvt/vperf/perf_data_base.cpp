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

#include "perf_data_base.h"

PerfDatabase* PerfDatabase::instance(void) {
  static PerfDatabase dataBase;

  return &dataBase;
}

TimeBlockData* ProcessorData::addTimeBlockData(TimeBlockData::Format f,
                                               const std::string& taskName,
                                               int eId, std::int64_t s,
                                               std::int64_t e, int tv) {
  TimeBlockData* data = new TimeBlockData(f, eId, s, e, tv);
  if (data == nullptr) return nullptr;

  TaskData* task;
  auto iter = _processorDatum.find(taskName);
  if (iter == _processorDatum.end()) {
    task = new TaskData();
    if (task == nullptr) {
      delete data;
      return nullptr;
    }

    _processorDatum.insert(std::make_pair(taskName, task));
  } else {
    task = iter->second;
  }

  task->insertTimeBlockData(data);

  return data;
}

TimeBlockData* PerfBlockDatabase::addTimeBlockData(TimeBlockData::Format f,
                                                   int processorIndex,
                                                   const std::string& taskName,
                                                   int eId, std::int64_t s,
                                                   std::int64_t e, int tv) {
  if (s > 0 && s < _minStartTimeStamp) {
    _minStartTimeStamp = s;
  }

  if (e > _maxEndTimeStamp) {
    _maxEndTimeStamp = e;
  }

  ProcessorData* procData = nullptr;
  int c = _datum.size();
  if (processorIndex < c) {
    procData = _datum.at(processorIndex);
  } else {
    ++processorIndex;  // to count;
    int i = c;
    for (; i < processorIndex; ++i) {
      ProcessorData* item = new ProcessorData();
      if (item) {
        _datum.push_back(item);
      } else {
        break;
      }
    }

    if (i == processorIndex) {
      procData = _datum.at(i - 1);
    } else {
      for (; i > c; --i) {
        ProcessorData* item = _datum.at(i - 1);
        _datum.pop_back();
        delete item;
      }
    }
  }

  if (procData) {
    return procData->addTimeBlockData(f, taskName, eId, s, e, tv);
  } else {
    return nullptr;
  }
}

PerfBlockDatabase* PerfDatabase::getSubDataBase(int index) {
  PerfBlockDatabase* ret = nullptr;
  int c = _datum.size();
  if (index < c) {
    ret = _datum.at(index);
  } else {
    ++index;  // to count
    int i = c;
    for (; i < index; ++i) {
      PerfBlockDatabase* item = new PerfBlockDatabase();
      if (item) {
        item->_blockIndex = i;
        _datum.push_back(item);
      } else {
        break;
      }
    }

    if (i == index) {
      ret = _datum.at(i - 1);
    } else {
      for (; i > c; --i) {
        PerfBlockDatabase* item = _datum.at(i - 1);
        _datum.pop_back();
        delete item;
      }
    }
  }

  return ret;
}
