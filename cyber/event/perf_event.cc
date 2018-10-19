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

#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>
#include <unordered_map>

#include "cyber/event/perf_event.h"

namespace apollo {
namespace cyber {
namespace event {

using apollo::cyber::base::BoundedQueue;
using apollo::cyber::common::GlobalData;

void SchedPerfEvent::SetParams(const int count, ...) {
  va_list ap;
  va_start(ap, count);
  event_id = va_arg(ap, int);
  cr_id = va_arg(ap, uint64_t);
  proc_id = va_arg(ap, int);
  t_sleep = va_arg(ap, uint64_t);
  t_start = va_arg(ap, uint64_t);
  t_end = va_arg(ap, uint64_t);
  try_fetch_result = va_arg(ap, int);
  croutine_state = va_arg(ap, int);
  va_end(ap);
}

void TransportPerfEvent::SetParams(const int count, ...) {
  va_list ap;
  va_start(ap, count);
  event_id = va_arg(ap, int);
  channel_id = va_arg(ap, uint64_t);
  msg_seq = va_arg(ap, uint64_t);
  t_end = va_arg(ap, uint64_t);
  va_end(ap);
}

}  // namespace event
}  // namespace cyber
}  // namespace apollo
