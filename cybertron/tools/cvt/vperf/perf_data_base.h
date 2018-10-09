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

#ifndef TOOLS_CVT_VPERF_PERFDATABASE_H_
#define TOOLS_CVT_VPERF_PERFDATABASE_H_

#include <cstdint>
#include <list>
#include <map>
#include <string>
#include <vector>

class TimeBlockData {
 public:
  enum class Format { Format8, Format11 };

  explicit TimeBlockData(Format f, int eId, std::int64_t s, std::int64_t e,
                         int tryFetchValue)
      : _format(f),
        _eventId(eId),
        _preEventId(-1),
        _tryFetchHolder(tryFetchValue),
        _latency(0),
        _startTimeStamp(s),
        _endTimeStamp(e),
        _preEndTimeStamp(0) {}

  std::int64_t startTimeStamp(void) const { return _startTimeStamp; }
  std::int64_t endTimeStamp(void) const { return _endTimeStamp; }
  int eventId(void) const { return _eventId; }

  void setPreEventId(int preEventId) { _preEventId = preEventId; }
  void setPreEndTime(std::int64_t t) { _preEndTimeStamp = t; }
  void setLatency(int l) { _latency = l; }

  int preEventId(void) const { return _preEventId; }
  int tryFetchHolderValue(void) const { return _tryFetchHolder; }
  std::int64_t preEndTime(void) const { return _preEndTimeStamp; }

  int latency(void) const { return _latency; }
  Format format(void) const { return _format; }

 private:
  Format _format;

  int _eventId;
  int _preEventId;
  int _tryFetchHolder;
  int _latency;
  std::int64_t _startTimeStamp;
  std::int64_t _endTimeStamp;
  std::int64_t _preEndTimeStamp;
};

class TaskData {
  std::list<TimeBlockData*> _datum;
  //    std::list<TimeBlockData*>::const_iterator _innerIter;

  TaskData(const TaskData&) = delete;
  TaskData& operator=(const TaskData&) = delete;

 public:
  explicit TaskData() : _datum() /*, _innerIter()*/ {
    //        _innerIter = _blockDatum.cbegin();
  }
  ~TaskData(void) {
    for (auto _innerIter = _datum.cbegin(); _innerIter != _datum.cend();
         ++_innerIter) {
      delete *_innerIter;
    }
    _datum.clear();
  }

  bool insertTimeBlockData(TimeBlockData::Format f, int eId, std::int64_t s,
                           std::int64_t e, int tv) {
    TimeBlockData* data = new TimeBlockData(f, eId, s, e, tv);
    return insertTimeBlockData(data);
  }

  bool insertTimeBlockData(TimeBlockData* data) {
    if (data == nullptr) return false;
    bool isCompleted = false;
    for (std::list<TimeBlockData*>::iterator iter = _datum.begin();
         iter != _datum.end(); ++iter) {
      if ((*iter)->endTimeStamp() > data->endTimeStamp()) {
        _datum.insert(iter, data);
        isCompleted = true;
        break;
      }
    }
    if (!isCompleted) {
      _datum.push_back(data);
    }

    return true;
  }

  const std::list<TimeBlockData*>& data(void) const { return _datum; }
  int timeBlockCount(void) const { return _datum.size(); }

  //    void resetTraverse(void){ _innerIter = _blockDatum.cbegin(); }
  //    bool hasNext(void)const{ return _innerIter != _blockDatum.cend(); }
  //    const TimeBlockData* currentTimeBlockData(void){
  //        const TimeBlockData* data = *_innerIter;
  //        ++_innerIter;
  //        return data;
  //    }
};

class ProcessorData {
  std::map<const std::string, TaskData*> _processorDatum;

 public:
  explicit ProcessorData(void) : _processorDatum() {}
  ~ProcessorData() {
    for (auto iter = _processorDatum.begin(); iter != _processorDatum.end();
         ++iter) {
      delete iter->second;
    }

    _processorDatum.clear();
  }

  TimeBlockData* addTimeBlockData(TimeBlockData::Format f,
                                  const std::string& taskName, int eId,
                                  std::int64_t s, std::int64_t e, int tv);
  TaskData* findTaskData(const std::string& taskName) {
    TaskData* ret = nullptr;
    auto iter = _processorDatum.find(taskName);
    if (iter != _processorDatum.end()) {
      ret = iter->second;
    }

    return ret;
  }

  const std::map<const std::string, TaskData*>& data(void) const {
    return _processorDatum;
  }
  int taskCount(void) const { return _processorDatum.size(); }
  int timeBlockCount(void) const {
    int s = 0;
    for (auto iter = _processorDatum.cbegin(); iter != _processorDatum.cend();
         ++iter) {
      s += iter->second->timeBlockCount();
    }
    return s;
  }
};

class PerfDatabase;

class PerfBlockDatabase {
  friend class PerfDatabase;

  int _blockIndex;
  std::int64_t _maxEndTimeStamp;
  std::int64_t _minStartTimeStamp;
  std::vector<ProcessorData*> _datum;

 public:
  explicit PerfBlockDatabase()
      : _blockIndex(-1),
        _maxEndTimeStamp(-1),
        _minStartTimeStamp(INT64_MAX),
        _datum() {}

  ~PerfBlockDatabase() { reset(); }

  ProcessorData* getProcessorData(int processorIndex) {
    if (processorIndex > -1 && (unsigned)processorIndex < _datum.size()) {
      return _datum.at(processorIndex);
    }
    return nullptr;
  }

  TimeBlockData* addTimeBlockData(TimeBlockData::Format f, int processorIndex,
                                  const std::string& taskName, int eId,
                                  std::int64_t s, std::int64_t e,
                                  int tryFetchValue);

  void reset(void) {
    _maxEndTimeStamp = -1;
    _minStartTimeStamp = INT64_MAX;
    for (auto item : _datum) {
      delete item;
    }
    _datum.clear();
  }

  const std::vector<ProcessorData*>& data(void) const { return _datum; }
  const ProcessorData* processorData(int index) const {
    return _datum.at(index);
  }

  std::int64_t endTimeStamp(void) const { return _maxEndTimeStamp; }
  std::int64_t startTimeStamp(void) const { return _minStartTimeStamp; }

  int processorCount(void) const { return _datum.size(); }

  int blockIndex(void) const { return _blockIndex; }

  int timeBlockCount(void) const {
    int s = 0;
    for (auto iter = _datum.cbegin(); iter != _datum.cend(); ++iter) {
      s += (*iter)->timeBlockCount();
    }
    return s;
  }
};

class PerfDatabase {
 public:
  static PerfDatabase* instance(void);

  PerfBlockDatabase* getSubDataBase(int index);

  void reset(void) {
    for (auto item : _datum) {
      item->reset();
    }
    _startTimeStamp = -1;
    _datum.clear();
  }

  int size(void) const { return _datum.size(); }

  void setStartTimeStamp(int64_t& t) { _startTimeStamp = t; }
  int64_t startTimeStamp(void) const { return _startTimeStamp; }

 private:
  PerfDatabase() : _startTimeStamp(-1), _datum() {}

  int64_t _startTimeStamp;
  std::vector<PerfBlockDatabase*> _datum;
};

#endif  // TOOLS_CVT_VPERF_PERFDATABASE_H_
