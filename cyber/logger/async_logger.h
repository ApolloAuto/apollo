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

#ifndef CYBER_LOGGER_ASYNC_LOGGER_H_
#define CYBER_LOGGER_ASYNC_LOGGER_H_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/common/macros.h"
#include "glog/logging.h"

#include "cyber/logger/log_file_object.h"

namespace apollo {
namespace cyber {
namespace logger {

/**
 * @class AsyncLogger
 * @brief .
 * Wrapper for a glog Logger which asynchronously writes log messages.
 * This class starts a new thread responsible for forwarding the messages
 * to the logger, and performs double buffering. Writers append to the
 * current buffer and then wake up the logger thread. The logger swaps in
 * a new buffer and writes any accumulated messages to the wrapped
 * Logger.
 *
 * This double-buffering design dramatically improves performance, especially
 * for logging messages which require flushing the underlying file (i.e WARNING
 * and above for default). The flush can take a couple of milliseconds, and in
 * some cases can even block for hundreds of milliseconds or more. With the
 * double-buffered approach, threads can proceed with useful work while the IO
 * thread blocks.
 *
 * The semantics provided by this wrapper are slightly weaker than the default
 * glog semantics. By default, glog will immediately (synchronously) flush
 * WARNING
 * and above to the underlying file, whereas here we are deferring that flush to
 * a separate thread. This means that a crash just after a 'LOG_WARN' would
 * may be missing the message in the logs, but the perf benefit is probably
 * worth it. We do take care that a glog FATAL message flushes all buffered log
 * messages before exiting.
 *
 * @warning The logger limits the total amount of buffer space, so if the
 * underlying log blocks for too long, eventually the threads generating the log
 * messages will block as well. This prevents runaway memory usage.
 */
class AsyncLogger : public google::base::Logger {
 public:
  explicit AsyncLogger(google::base::Logger* wrapped);

  ~AsyncLogger();

  /**
   * @brief start the async logger
   */
  void Start();

  /**
   * @brief Stop the thread. Flush() and Write() must not be called after this.
   * NOTE: this is currently only used in tests: in real life, we enable async
   * logging once when the program starts and then never disable it.
   * REQUIRES: Start() must have been called.
   */
  void Stop();

  /**
   * @brief Write a message to the log. Start() must have been called.
   *
   * @param force_flush is set by the GLog library based on the configured
   * '--logbuflevel' flag.
   * Any messages logged at the configured level or higher result in
   * 'force_flush' being set to true, indicating that the message should be
   * immediately written to the log rather than buffered in memory.
   * @param timestamp is the time of write a message
   * @param message is the info to be written
   * @param message_len is the length of message
   */
  void Write(bool force_flush, time_t timestamp, const char* message,
             int message_len) override;

  /**
   * @brief Flush any buffered messages.
   */
  void Flush() override;

  /**
   * @brief Get the current LOG file size.
   * The return value is an approximate value since some
   * logged data may not have been flushed to disk yet.
   *
   * @return the log file size
   */
  uint32_t LogSize() override;

  /**
   * @brief get the log thead
   *
   * @return the pointer of log thread
   */
  std::thread* LogThread() { return &log_thread_; }

 private:
  // A buffered message.
  //
  // TODO(todd): using std::string for buffered messages is convenient but not
  // as efficient as it could be. It's better to make the buffers just be
  // Arenas and allocate both the message data and Msg struct from them, forming
  // a linked list.
  struct Msg {
    time_t ts;
    std::string message;
    int32_t level;
    Msg() : ts(0), message(), level(google::INFO) {}
    Msg(time_t ts, std::string&& message, int32_t level)
        : ts(ts), message(std::move(message)), level(level) {}
    Msg(const Msg& rsh) {
      ts = rsh.ts;
      message = rsh.message;
      level = rsh.level;
    }
    Msg(Msg&& rsh) {
      ts = rsh.ts;
      message = rsh.message;
      level = rsh.level;
    }
    Msg& operator=(Msg&& rsh) {
      ts = rsh.ts;
      message = std::move(rsh.message);
      level = rsh.level;
      return *this;
    }
    Msg& operator=(const Msg& rsh) {
      ts = rsh.ts;
      message = rsh.message;
      level = rsh.level;
      return *this;
    }
  };

  void RunThread();
  void FlushBuffer(const std::unique_ptr<std::deque<Msg>>& msg);

  google::base::Logger* const wrapped_;
  std::thread log_thread_;

  // Count of how many times the writer thread has flushed the buffers.
  // 64 bits should be enough to never worry about overflow.
  std::atomic<uint64_t> flush_count_ = {0};

  // Count of how many times the writer thread has dropped the log messages.
  // 64 bits should be enough to never worry about overflow.
  uint64_t drop_count_ = 0;

  // The buffer to which application threads append new log messages.
  std::unique_ptr<std::deque<Msg>> active_buf_;

  // The buffer currently being flushed by the logger thread, cleared
  // after a successful flush.
  std::unique_ptr<std::deque<Msg>> flushing_buf_;

  // Trigger for the logger thread to stop.
  enum State { INITTED, RUNNING, STOPPED };
  std::atomic<State> state_ = {INITTED};
  std::atomic_flag flag_ = ATOMIC_FLAG_INIT;
  std::unordered_map<std::string, std::unique_ptr<LogFileObject>>
      module_logger_map_;

  DISALLOW_COPY_AND_ASSIGN(AsyncLogger);
};

}  // namespace logger
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_LOGGER_ASYNC_LOGGER_H_
