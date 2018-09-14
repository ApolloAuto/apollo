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

#ifndef CYBERTRON_PERF_EVENT_CACHE_H_
#define CYBERTRON_PERF_EVENT_CACHE_H_

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <thread>
#include <unordered_map>

#include "cybertron/base/bounded_queue.h"
#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/common/macros.h"
#include "cybertron/event/perf_event.h"
#include "cybertron/proto/perf_conf.pb.h"

#define MAX_EVENT_SIZE 16384

namespace apollo {
namespace cybertron {
namespace event {

using apollo::cybertron::base::BoundedQueue;
using apollo::cybertron::common::GlobalData;
using apollo::cybertron::proto::PerfConf;

class PerfEventCache {
 public:
  ~PerfEventCache();
  void AddSchedEvent(SchedPerf event_id, uint64_t cr_id, int proc_id,
                     uint64_t t_sleep, uint64_t t_start, int try_fetch_result,
                     int croutine_state);
  void AddTransportEvent(TransPerf event_id, uint64_t channel_id,
                         uint64_t msg_seq);
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
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_INIT_H_
