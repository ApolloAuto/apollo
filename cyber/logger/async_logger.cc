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

#include "cyber/logger/async_logger.h"

#include <cstdlib>
#include <string>
#include <thread>
#include <unordered_map>

#include "cyber/base/macros.h"
#include "cyber/logger/log_file_object.h"
#include "cyber/logger/logger_util.h"

namespace apollo {
namespace cyber {
namespace logger {

static std::unordered_map<std::string, LogFileObject*> moduleLoggerMap;
static std::unordered_map<char, int> log_level_map = {
    {'F', 3}, {'E', 2}, {'W', 1}, {'I', 0}};

AsyncLogger::AsyncLogger(google::base::Logger* wrapped) : wrapped_(wrapped) {
  active_buf_.reset(new std::deque<Msg>());
  flushing_buf_.reset(new std::deque<Msg>());
}

AsyncLogger::~AsyncLogger() {
  for (auto& logger : moduleLoggerMap) {
    delete logger.second;
  }
  moduleLoggerMap.clear();
}

void AsyncLogger::Start() {
  CHECK_EQ(state_.load(std::memory_order_acquire), INITTED);
  state_.store(RUNNING, std::memory_order_release);
  log_thread_ = std::thread(&AsyncLogger::RunThread, this);
  // std::cout << "Async Logger Start!" << std::endl;
}

void AsyncLogger::Stop() {
  CHECK_EQ(state_.load(std::memory_order_acquire), RUNNING);
  state_.store(STOPPED, std::memory_order_release);
  if (log_thread_.joinable()) {
    log_thread_.join();
  }

  FlushBuffer(active_buf_);
  CHECK(active_buf_->empty());
  CHECK(flushing_buf_->empty());
  // std::cout << "Async Logger Stop!" << std::endl;
}

void AsyncLogger::Write(bool force_flush, time_t timestamp, const char* message,
                        int message_len) {
  (void)force_flush;
  if (cyber_unlikely(state_.load(std::memory_order_acquire) != RUNNING)) {
    // std::cout << "Async Logger not running!" << std::endl;
    return;
  }
  auto msg_str = std::string(message, message_len);
  while (flag_.test_and_set(std::memory_order_acquire)) {
    cpu_relax();
  }
  active_buf_->emplace_back(timestamp, std::move(msg_str),
                            log_level_map[message[0]]);
  flag_.clear(std::memory_order_release);
}

void AsyncLogger::Flush() {
  for (auto& module_logger : moduleLoggerMap) {
    module_logger.second->Flush();
  }
}

uint32_t AsyncLogger::LogSize() { return wrapped_->LogSize(); }

void AsyncLogger::RunThread() {
  while (state_ == RUNNING) {
    while (flag_.test_and_set(std::memory_order_acquire)) {
      cpu_relax();
    }
    active_buf_.swap(flushing_buf_);
    flag_.clear(std::memory_order_release);
    FlushBuffer(flushing_buf_);
    if (active_buf_->size() < 800) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void AsyncLogger::FlushBuffer(const std::unique_ptr<std::deque<Msg>>& buffer) {
  std::string module_name = "";
  while (!buffer->empty()) {
    auto& msg = buffer->front();
    FindModuleName(&(msg.message), &module_name);

    LogFileObject* fileobject = nullptr;
    if (moduleLoggerMap.find(module_name) != moduleLoggerMap.end()) {
      fileobject = moduleLoggerMap[module_name];
    } else {
      std::string file_name = module_name + ".log.INFO.";
      if (!FLAGS_log_dir.empty()) {
        file_name = FLAGS_log_dir + "/" + file_name;
      }
      fileobject = new LogFileObject(google::INFO, file_name.c_str());
      fileobject->SetSymlinkBasename(module_name.c_str());
      moduleLoggerMap[module_name] = fileobject;
    }
    if (fileobject) {
      const bool force_flush = msg.level > 0;
      fileobject->Write(force_flush, msg.ts, msg.message.data(),
                        static_cast<int>(msg.message.size()));
    }
    buffer->pop_front();
  }
  Flush();
}

}  // namespace logger
}  // namespace cyber
}  // namespace apollo
