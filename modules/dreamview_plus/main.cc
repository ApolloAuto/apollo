/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "gperftools/profiler.h"
#include "gperftools/heap-profiler.h"
#include "gperftools/malloc_extension.h"

#include "cyber/common/global_data.h"
#include "cyber/init.h"
#include "modules/dreamview_plus/backend/dreamview.h"

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Added by caros to improve dv performance

  std::signal(SIGTERM, [](int sig){
    apollo::cyber::OnShutdown(sig);
    if (FLAGS_dv_cpu_profile) {
      ProfilerStop();
    }
    if (FLAGS_dv_heap_profile) {
      HeapProfilerDump("Befor shutdown");
      HeapProfilerStop();
    }
  });

  std::signal(SIGINT, [](int sig){
    apollo::cyber::OnShutdown(sig);
    if (FLAGS_dv_cpu_profile) {
      ProfilerStop();
    }
    if (FLAGS_dv_heap_profile) {
      HeapProfilerDump("Befor shutdown");
      HeapProfilerStop();
    }
  });

  if (FLAGS_dv_cpu_profile) {
    auto base_name_cpu = std::string(argv[0]) + "_cpu" + std::string(".prof");
    ProfilerStart(base_name_cpu.c_str());
  }
  if (FLAGS_dv_heap_profile) {
    auto base_name_heap = std::string(argv[0]) + "_heap" + std::string(".prof");
    HeapProfilerStart(base_name_heap.c_str());
  }

  apollo::cyber::GlobalData::Instance()->SetProcessGroup("dreamview_sched");
  apollo::cyber::Init(argv[0]);

  apollo::dreamview::Dreamview dreamview;
  const bool init_success = dreamview.Init().ok() && dreamview.Start().ok();
  if (!init_success) {
    AERROR << "Failed to initialize dreamview server";
    return -1;
  }
  apollo::cyber::WaitForShutdown();
  return 0;
}
