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

#include "cyber/init.h"

#include <libgen.h>
#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <csignal>
#include <string>

#include "cyber/binary.h"
#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/global_data.h"
#include "cyber/data/data_dispatcher.h"
#include "cyber/event/perf_event_cache.h"
#include "cyber/logger/async_logger.h"
#include "cyber/node/node.h"
#include "cyber/scheduler/scheduler.h"
#include "cyber/service_discovery/topology_manager.h"
#include "cyber/task/task.h"
#include "cyber/transport/transport.h"

namespace apollo {
namespace cyber {

using apollo::cyber::common::EnsureDirectory;
using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::GetProtoFromFile;
using apollo::cyber::common::WorkRoot;
using apollo::cyber::croutine::CRoutine;
using apollo::cyber::event::PerfEventCache;
using apollo::cyber::scheduler::Scheduler;
using apollo::cyber::service_discovery::TopologyManager;

static bool g_atexit_registered = false;
// TODO(hewei03): why not use simple mutex?
static std::recursive_mutex g_mutex;

static ::apollo::cyber::logger::AsyncLogger* async_logger;

void OnShutdown(int sig) {
  (void)sig;
  if (GetState() != STATE_SHUTDOWN) {
    SetState(STATE_SHUTTING_DOWN);
  }
}

void ExitHandle() {
  AINFO << "Shutdown in ExitHandle";
  Shutdown();
}

bool Init() {
  std::lock_guard<std::recursive_mutex> lg(g_mutex);
  if (GetState() != STATE_UNINITIALIZED) {
    // avoid reinit
    return false;
  }

  std::signal(SIGINT, OnShutdown);

  // Initialize internal static objects
  common::GlobalData::Instance();
  transport::Transport::Instance();
  service_discovery::TopologyManager::Instance();
  scheduler::Scheduler::Instance();
  TaskManager::Instance();
  PerfEventCache::Instance();

  // Register exit handlers
  if (!g_atexit_registered) {
    if (std::atexit(ExitHandle) != 0) {
      AERROR << "Register exit handle failed";
      return false;
    }
    AINFO << "Register exit handle succ.";
    g_atexit_registered = true;
  }
  SetState(STATE_INITIALIZED);
  return true;
}

bool Init(const char* argv) {
  const char* slash = strrchr(argv, '/');
  if (slash) {
    ::apollo::cyber::Binary::SetName(slash + 1);
  } else {
    ::apollo::cyber::Binary::SetName(argv);
  }

  // Get log conf object
  auto& log_conf = common::GlobalData::Instance()->Config().log_conf();

  // Ensure log dir
  auto& log_dir = log_conf.log_dir();
  std::string abs_log_dir(log_dir);
  if (log_dir[0] != '/') {
    abs_log_dir = GetAbsolutePath(WorkRoot(), log_dir);
  }
  EnsureDirectory(abs_log_dir);

  // Set flags
  FLAGS_log_dir = abs_log_dir;
  if (log_conf.min_log_level() >= google::INFO) {
    FLAGS_minloglevel = log_conf.min_log_level();
  } else {
    FLAGS_minloglevel = google::INFO;
    FLAGS_v = 4;
  }
  FLAGS_alsologtostderr = log_conf.log_to_stderr();
  FLAGS_colorlogtostderr = log_conf.color_log_to_stderr();
  google::InitGoogleLogging(argv);
  google::SetLogDestination(google::ERROR, "");
  google::SetLogDestination(google::WARNING, "");
  google::SetLogDestination(google::FATAL, "");

  async_logger = new ::apollo::cyber::logger::AsyncLogger(
      google::base::GetLogger(FLAGS_minloglevel), 2 * 1024 * 1024);
  google::base::SetLogger(FLAGS_minloglevel, async_logger);
  async_logger->Start();

  AINFO << "glog inited";
  AINFO << "glog FLAGS_log_dir=" << FLAGS_log_dir;
  AINFO << "glog FLAGS_minloglevel=" << FLAGS_minloglevel;
  AINFO << "glog FLAGS_alsologtostderr=" << FLAGS_alsologtostderr;
  AINFO << "glog FLAGS_colorlogtostderr=" << FLAGS_colorlogtostderr;

  return Init();
}

void Shutdown() {
  std::lock_guard<std::recursive_mutex> lg(g_mutex);
  if (GetState() == STATE_SHUTDOWN) {
    return;
  }
  TaskManager::Instance()->Shutdown();
  scheduler::Scheduler::Instance()->ShutDown();
  service_discovery::TopologyManager::Instance()->Shutdown();
  transport::Transport::Instance()->Shutdown();
  SetState(STATE_SHUTDOWN);
}

}  // namespace cyber
}  // namespace apollo
