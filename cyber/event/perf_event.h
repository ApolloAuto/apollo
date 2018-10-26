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

#ifndef CYBER_EVENT_PERF_EVENT_H_
#define CYBER_EVENT_PERF_EVENT_H_

#include <chrono>
#include <cstdarg>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include "cyber/base/bounded_queue.h"
#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"

namespace apollo {
namespace cyber {
namespace event {

using apollo::cyber::base::BoundedQueue;
using apollo::cyber::common::GlobalData;

enum class EventType { SCHED_EVENT = 0, TRANS_EVENT = 1, TRY_FETCH_EVENT = 3 };

enum class TransPerf { TRANS_FROM = 1, TRANS_TO = 2, WRITE_NOTIFY = 3 };

enum class SchedPerf {
  SWAP_IN = 1,
  SWAP_OUT = 2,
  NOTIFY_IN = 3,
  NEXT_RT = 4,
  RT_CREATE = 5,
};

class EventBase {
 public:
  virtual std::string SerializeToString() = 0;

  inline void set_eid(int eid) {eid_ = eid;}
  inline void set_etype(int etype) {etype_ = etype;}
  inline void set_stamp(uint64_t stamp) {stamp_ = stamp;}

  virtual void set_cr_id(uint64_t cr_id) {}
  virtual void set_cr_state(int cr_state) {}
  virtual void set_proc_id(int proc_id) {}
  virtual void set_fetch_res_(int fetch_res) {}

  virtual void set_msg_seq(const uint64_t msg_seq) {}
  virtual void set_channel_id(const uint64_t channel_id) {}

 protected:
  int etype_;
  int eid_;
  uint64_t stamp_;
};

// event_id
// 1 swap_in
// 2 swap_out
// 3 notify_in
// 4 next_routine
class SchedEvent : public EventBase {
 public:
  SchedEvent() {
    etype_ = static_cast<int>(EventType::SCHED_EVENT);
  }

  virtual std::string SerializeToString() {
    std::stringstream ss;
    ss << etype_ << "\t";
    ss << eid_ << "\t";
    ss << apollo::cyber::common::
        GlobalData::GetTaskNameById(cr_id_) << "\t";
    ss << proc_id_ << "\t";
    ss << cr_state_ << "\t";
    ss << stamp_;
    return ss.str();
  }

  inline void set_cr_id(uint64_t cr_id) override {cr_id_ = cr_id;}
  inline void set_cr_state(int cr_state) override {cr_state_ = cr_state;}
  inline void set_proc_id(int proc_id) override {proc_id_ = proc_id;}

 private:
  int cr_state_;
  int proc_id_;
  uint64_t cr_id_;
};

// event_id = 1 transport
// 1 transport time
// 2 write_data_cache & notify listener
class TransportEvent : public EventBase {
 public:
  TransportEvent() {
    etype_ = static_cast<int>(EventType::TRANS_EVENT);
  }

  std::string SerializeToString() override {
    std::stringstream ss;
    ss << etype_ << "\t";
    ss << eid_ << "\t";
    ss << apollo::cyber::common::
        GlobalData::GetChannelById(channel_id_) << "\t";
    ss << msg_seq_ << "\t";
    ss << stamp_;
    return ss.str();
  }

  inline void set_msg_seq(const uint64_t msg_seq) override {msg_seq_ = msg_seq;}
  inline void set_channel_id(const uint64_t channel_id) override {
    channel_id_ = channel_id;
  }

 private:
  uint64_t msg_seq_ = 0;
  uint64_t channel_id_ = -1;
};

}  // namespace event
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_EVENT_PERF_EVENT_H_
