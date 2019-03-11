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

#include "cyber/base/macros.h"
#include "cyber/common/environment.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/state.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace event {

using common::GetEnv;

PerfEventCache::PerfEventCache() {
  auto trans_perf = GetEnv("cyber_trans_perf");
  if (trans_perf != "" && std::stoi(trans_perf)) {
    enable_trans_perf_ = true;
  }
  auto sched_perf = GetEnv("cyber_sched_perf");
  if (sched_perf != "" && std::stoi(sched_perf)) {
    enable_sched_perf_ = true;
  }

  if (enable_sched_perf_ || enable_trans_perf_) {
    if (!event_queue_.Init(kEventQueueSize)) {
      AERROR << "Event queue init failed.";
      throw std::runtime_error("Event queue init failed.");
    }
    Start();
  }
}

PerfEventCache::~PerfEventCache() {
  if (!enable_sched_perf_ && !enable_trans_perf_) {
    return;
  }

  if (shutdown_.exchange(true)) {
    return;
  }

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
  if (likely(!enable_sched_perf_)) {
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
                                       const uint64_t msg_seq) {
  if (likely(!enable_trans_perf_)) {
    return;
  }

  EventBasePtr e = std::make_shared<TransportEvent>();
  e->set_eid(static_cast<int>(event_id));
  e->set_channel_id(channel_id);
  e->set_msg_seq(msg_seq);
  e->set_stamp(Time::Now().ToNanosecond());

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
  of_.open(perf_file, std::ios::trunc);
  of_ << Time::Now().ToNanosecond() << std::endl;
  io_thread_ = std::thread(&PerfEventCache::Run, this);
}

}  // namespace event
}  // namespace cyber
}  // namespace apollo
