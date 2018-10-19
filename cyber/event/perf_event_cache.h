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

#ifndef CYBER_PERF_EVENT_CACHE_H_
#define CYBER_PERF_EVENT_CACHE_H_

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <thread>
#include <unordered_map>

#include "cyber/base/bounded_queue.h"
#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/event/perf_event.h"
#include "cyber/proto/perf_conf.pb.h"

#define MAX_EVENT_SIZE 16384

namespace apollo {
namespace cyber {
namespace event {

using apollo::cyber::base::BoundedQueue;
using apollo::cyber::common::GlobalData;
using apollo::cyber::proto::PerfConf;

class PerfEventCache {
 public:
  ~PerfEventCache();
  void AddSchedEvent(const SchedPerf event_id, const uint64_t cr_id,
                     const int proc_id, const uint64_t t_sleep,
                     const uint64_t t_start, const int try_fetch_result,
                     const int croutine_state);
  void AddTransportEvent(const TransPerf event_id, const uint64_t channel_id,
                         const uint64_t msg_seq);
  void AddEvent(const std::shared_ptr<PerfEventBase>& event);

 private:
  void Start();
  void Run();
  BoundedQueue<std::shared_ptr<PerfEventBase>> event_queue_;

  bool enable_ = false;
  bool shutdown_ = false;
  PerfConf perf_conf_;
  std::thread io_thread_;
  std::ofstream of_;
  DECLARE_SINGLETON(PerfEventCache)
};
}  // namespace event
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_INIT_H_
