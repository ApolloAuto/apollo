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

#ifndef CYBERTRON_EVENT_PERF_EVENT_H_
#define CYBERTRON_EVENT_PERF_EVENT_H_

#include <chrono>
#include <cstdarg>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include "cybertron/base/bounded_queue.h"
#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/common/macros.h"

#define MAX_EVENT_SIZE 16384

namespace apollo {
namespace cybertron {
namespace event {

using apollo::cybertron::base::BoundedQueue;
using apollo::cybertron::common::GlobalData;

enum class EventType { SCHED_EVENT = 0, TRANS_EVENT = 1 };

enum class SchedPerf {
  SWAP_IN = 1,
  SWAP_OUT = 2,
  TRY_FETCH_OUT = 3,
  NOTIFY_IN = 4,
  NEXT_ROUTINE = 5
};

enum class TransPerf { TRANS_FROM = 1, TRANS_TO = 2, WRITE_NOTIFY = 3 };

// event_id
// 1 swap_in
// 2 swap_out
// 3 try_fetch_out
// 4 notify_in
// 5 next_routine

class PerfEventBase {
 public:
  int event_type = 0;
  int event_id;
  uint64_t t_start = -1;
  uint64_t t_end = -1;
  virtual void SetParams(int count, ...) {
    va_list ap;
    va_start(ap, count);
    event_type = va_arg(ap, int);
    event_id = va_arg(ap, int);
    t_start = va_arg(ap, uint64_t);
    t_end = va_arg(ap, uint64_t);
    va_end(ap);
  }
  virtual std::string SerializeToString() {
    std::stringstream ss;
    ss << event_type << "\t";
    ss << event_id << "\t";
    ss << t_start << "\t";
    ss << t_end;
    return ss.str();
  }
};

// event_id
// 1 swap_in
// 2 swap_out
// 3 try_fetch_out
// 4 notify_in
// 5 next_routine
class SchedPerfEvent : public PerfEventBase {
 public:
  SchedPerfEvent() { event_type = int(EventType::SCHED_EVENT); }
  void SetParams(int count, ...) override;
  std::string SerializeToString() override {
    std::stringstream ss;
    ss << event_type << "\t";
    ss << event_id << "\t";
    ss << apollo::cybertron::common::GlobalData::GetTaskNameById(cr_id) << "\t";
    ss << proc_id << "\t";
    ss << t_sleep << "\t";
    ss << t_start << "\t";
    ss << t_end << "\t";
    ss << try_fetch_result << "\t";
    ss << croutine_state;
    return ss.str();
  }

 private:
  uint64_t cr_id = -1;
  int proc_id = -1;
  uint64_t t_sleep = 0;
  int try_fetch_result = -1;
  int croutine_state = -1;
};

// event_id = 1 transport
// 1 transport time
// 2 write_data_cache & notify listener
class TransportPerfEvent : public PerfEventBase {
 public:
  TransportPerfEvent() { event_type = int(EventType::TRANS_EVENT); }
  void SetParams(int count, ...) override;
  std::string SerializeToString() override {
    std::stringstream ss;
    ss << event_type << "\t";
    ss << event_id << "\t";
    ss << apollo::cybertron::common::GlobalData::GetChannelById(channel_id)
       << "\t";
    ss << msg_seq << "\t";
    ss << t_end;
    return ss.str();
  }

 private:
  uint64_t msg_seq = 0;
  uint64_t channel_id = -1;
};

}  // namespace event
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_EVENT_PERF_EVENT_H_
