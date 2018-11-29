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
static std::recursive_mutex g_mutex;

static ::apollo::cyber::logger::AsyncLogger* async_logger;

namespace {

void InitLogger(const char* binary_name) {
  const char* slash = strrchr(binary_name, '/');
  if (slash) {
    ::apollo::cyber::Binary::SetName(slash + 1);
  } else {
    ::apollo::cyber::Binary::SetName(binary_name);
  }
  CHECK_NOTNULL(common::GlobalData::Instance());
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
  google::InitGoogleLogging(binary_name);
  google::SetLogDestination(google::ERROR, "");
  google::SetLogDestination(google::WARNING, "");
  google::SetLogDestination(google::FATAL, "");

  async_logger = new ::apollo::cyber::logger::AsyncLogger(
      google::base::GetLogger(FLAGS_minloglevel), 2 * 1024 * 1024);
  google::base::SetLogger(FLAGS_minloglevel, async_logger);
  async_logger->Start();

  ADEBUG << "glog inited";
  ADEBUG << "glog FLAGS_log_dir=" << FLAGS_log_dir;
  ADEBUG << "glog FLAGS_minloglevel=" << FLAGS_minloglevel;
  ADEBUG << "glog FLAGS_alsologtostderr=" << FLAGS_alsologtostderr;
  ADEBUG << "glog FLAGS_colorlogtostderr=" << FLAGS_colorlogtostderr;
}

void CheckSingleton() {
  // Initialize internal static objects
  CHECK_NOTNULL(transport::Transport::Instance());
  CHECK_NOTNULL(service_discovery::TopologyManager::Instance());
  CHECK_NOTNULL(scheduler::Instance());
  CHECK_NOTNULL(TaskManager::Instance());
  CHECK_NOTNULL(PerfEventCache::Instance());
}

void StopLogger() {
  if (async_logger != nullptr) {
    async_logger->Stop();
  }
}
}  // namespace

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

bool Init(const char* binary_name) {
  std::lock_guard<std::recursive_mutex> lg(g_mutex);
  // avoid reinit
  if (GetState() != STATE_UNINITIALIZED) {
    return false;
  }

  InitLogger(binary_name);
  CheckSingleton();
  std::signal(SIGINT, OnShutdown);
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

void Shutdown() {
  std::lock_guard<std::recursive_mutex> lg(g_mutex);
  if (GetState() == STATE_SHUTDOWN || GetState() == STATE_UNINITIALIZED) {
    return;
  }
  TaskManager::Instance()->Shutdown();
  scheduler::Instance()->ShutDown();
  service_discovery::TopologyManager::Instance()->Shutdown();
  transport::Transport::Instance()->Shutdown();
  StopLogger();
  SetState(STATE_SHUTDOWN);
}

}  // namespace cyber
}  // namespace apollo
