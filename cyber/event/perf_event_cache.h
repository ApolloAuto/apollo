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

#ifndef CYBER_EVENT_CACHE_H_
#define CYBER_EVENT_CACHE_H_

#include <chrono>
#include <fstream>
#include <memory>
#include <thread>

#include "cyber/base/bounded_queue.h"
#include "cyber/common/macros.h"
#include "cyber/event/perf_event.h"

namespace apollo {
namespace cyber {
namespace event {

class PerfEventCache {
 public:
  using EventBasePtr = std::shared_ptr<EventBase>;

  ~PerfEventCache();
  void AddSchedEvent(const SchedPerf event_id, const uint64_t cr_id,
                     const int proc_id, const int cr_state = -1);
  void AddTransportEvent(const TransPerf event_id, const uint64_t channel_id,
                         const uint64_t msg_seq);

 private:
  void Start();
  void Run();

  std::thread io_thread_;
  std::ofstream of_;

  bool enable_trans_perf_ = false;
  bool enable_sched_perf_ = false;
  std::atomic<bool> shutdown_ = {false};

  base::BoundedQueue<EventBasePtr> event_queue_;

  const int kFlushSize = 512;
  const uint64_t kEventQueueSize = 8192;

  DECLARE_SINGLETON(PerfEventCache)
};
}  // namespace event
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_INIT_H_
