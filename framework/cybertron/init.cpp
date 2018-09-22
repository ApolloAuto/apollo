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

#include "cybertron/init.h"

#include <libgen.h>
#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <csignal>
#include <string>

#include "cybertron/binary.h"
#include "cybertron/common/environment.h"
#include "cybertron/common/file.h"
#include "cybertron/common/global_data.h"
#include "cybertron/data/data_dispatcher.h"
#include "cybertron/event/perf_event_cache.h"
#include "cybertron/logger/async_logger.h"
#include "cybertron/node/node.h"
#include "cybertron/scheduler/scheduler.h"
#include "cybertron/service_discovery/topology_manager.h"
#include "cybertron/task/task.h"

namespace apollo {
namespace cybertron {

using apollo::cybertron::common::EnsureDirectory;
using apollo::cybertron::common::GetAbsolutePath;
using apollo::cybertron::common::GetProtoFromFile;
using apollo::cybertron::common::WorkRoot;
using apollo::cybertron::croutine::CRoutine;
using apollo::cybertron::scheduler::Scheduler;
using apollo::cybertron::service_discovery::TopologyManager;
using apollo::cybertron::event::PerfEventCache;

static bool g_ok = false;
static bool g_is_shutdown = false;
static bool g_atexit_registered = false;
// TODO(hewei03): why not use simple mutex?
static std::recursive_mutex g_mutex;

static ::apollo::cybertron::logger::AsyncLogger* async_logger;

void OnShutdown(int signal) {
  // TODO(hewei03)
  if (!IsShutdown()) {
    AINFO << "Shutdown with signal interrupt.";
    Shutdown();
  }
}

void ExitHandle() {
  if (OK() && !IsShutdown()) {
    AINFO << "Shutdown in ExitHandle";
    Shutdown();
  }
}

bool Init() {
  std::lock_guard<std::recursive_mutex> lg(g_mutex);
  if (g_ok) {
    // avoid reinit
    return true;
  }
  if (g_is_shutdown) {
    // cybertron is shutdown
    return false;
  }

  // Initialize internal static objects
  common::GlobalData::Instance();
  service_discovery::TopologyManager::Instance();
  scheduler::Scheduler::Instance();
  TaskManager::Instance();
  PerfEventCache::Instance();

  // Register exit handlers
  std::signal(SIGINT, OnShutdown);
  if (!g_atexit_registered) {
    if (std::atexit(ExitHandle) != 0) {
      AERROR << "Register exit handle failed";
      return false;
    }
    AINFO << "Register exit handle succ.";
    g_atexit_registered = true;
  }
  g_ok = true;
  return true;
}

bool Init(const char* argv) {
  const char* slash = strrchr(argv, '/');
  if (slash) {
    ::apollo::cybertron::Binary::SetName(slash + 1);
  } else {
    ::apollo::cybertron::Binary::SetName(argv);
  }

  // Get log conf object
  auto log_conf = common::GlobalData::Instance()->Config().log_conf();

  // Ensure log dir
  std::string log_dir = log_conf.log_dir();
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

  async_logger = new ::apollo::cybertron::logger::AsyncLogger(
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

bool OK() { return g_ok; }

void Shutdown() {
  std::lock_guard<std::recursive_mutex> lg(g_mutex);
  if (g_is_shutdown) {
    return;
  }
  scheduler::Scheduler::Instance()->ShutDown();
  service_discovery::TopologyManager::Instance()->Shutdown();
  g_is_shutdown = true;
  g_ok = false;
}

void AsyncShutdown() {
  pid_t pid = getpid();
  if (kill(pid, SIGINT) != 0) {
    AERROR << strerror(errno);
  }
}

bool IsShutdown() { return g_is_shutdown; }

void WaitForShutdown() {
  while (OK() && !IsShutdown()) {
    usleep(200 * 1000);
  }
}

// Internal interface for debug.
void PrintSchedulerStatistics() { Scheduler::Instance()->PrintStatistics(); }

}  // namespace cybertron
}  // namespace apollo
