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

#include "cyber/event/perf_event_cache.h"

#include <string>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/state.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace event {

using common::GlobalData;
using proto::PerfConf;
using proto::PerfType;

PerfEventCache::PerfEventCache() {
  auto& global_conf = GlobalData::Instance()->Config();
  if (global_conf.has_perf_conf()) {
    perf_conf_.CopyFrom(global_conf.perf_conf());
    enable_ = perf_conf_.enable();
  }

  if (enable_) {
    if (!event_queue_.Init(kEventQueueSize)) {
      AERROR << "Event queue init failed.";
      throw std::runtime_error("Event queue init failed.");
    }
    Start();
  }
}

PerfEventCache::~PerfEventCache() { Shutdown(); }

void PerfEventCache::Shutdown() {
  if (!enable_) {
    return;
  }

  shutdown_ = true;
  event_queue_.BreakAllWait();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }

  of_ << cyber::Time::Now().ToNanosecond() << std::endl;
  of_.flush();
  of_.close();
}

void PerfEventCache::AddSchedEvent(const SchedPerf event_id,
                                   const uint64_t cr_id, const int proc_id,
                                   const int cr_state) {
  if (!enable_) {
    return;
  }

  if (perf_conf_.type() != PerfType::SCHED &&
      perf_conf_.type() != PerfType::ALL) {
    return;
  }

  EventBasePtr e = std::make_shared<SchedEvent>();
  e->set_eid(static_cast<int>(event_id));
  e->set_stamp(Time::Now().ToNanosecond());
  e->set_cr_state(cr_state);
  e->set_cr_id(cr_id);
  e->set_proc_id(proc_id);

  event_queue_.Enqueue(e);
}

void PerfEventCache::AddTransportEvent(const TransPerf event_id,
                                       const uint64_t channel_id,
                                       const uint64_t msg_seq,
                                       const uint64_t stamp,
                                       const std::string& adder) {
  if (!enable_) {
    return;
  }

  if (perf_conf_.type() != PerfType::TRANSPORT &&
      perf_conf_.type() != PerfType::ALL) {
    return;
  }

  EventBasePtr e = std::make_shared<TransportEvent>();
  e->set_eid(static_cast<int>(event_id));
  e->set_channel_id(channel_id);
  e->set_msg_seq(msg_seq);
  e->set_adder(adder);
  auto t = stamp;
  if (stamp == 0) {
    t = Time::Now().ToNanosecond();
  }
  e->set_stamp(t);

  event_queue_.Enqueue(e);
}

void PerfEventCache::Run() {
  EventBasePtr event;
  int buf_size = 0;
  while (!shutdown_ && !apollo::cyber::IsShutdown()) {
    if (event_queue_.WaitDequeue(&event)) {
      of_ << event->SerializeToString() << std::endl;
      buf_size++;
      if (buf_size >= kFlushSize) {
        of_.flush();
        buf_size = 0;
      }
    }
  }
}

void PerfEventCache::Start() {
  auto now = Time::Now();
  std::string perf_file = "cyber_perf_" + now.ToString() + ".data";
  std::replace(perf_file.begin(), perf_file.end(), ' ', '_');
  std::replace(perf_file.begin(), perf_file.end(), ':', '-');
  of_.open(perf_file, std::ios::trunc);
  perf_file_ = perf_file;
  of_ << Time::Now().ToNanosecond() << std::endl;
  io_thread_ = std::thread(&PerfEventCache::Run, this);
}

}  // namespace event
}  // namespace cyber
}  // namespace apollo
