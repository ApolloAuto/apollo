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

#ifndef CYBER_LOGGER_LOG_FILE_OBJECT_H_
#define CYBER_LOGGER_LOG_FILE_OBJECT_H_

#include <cstdint>
#include <mutex>
#include <string>

#include "glog/logging.h"

namespace apollo {
namespace cyber {
namespace logger {

class LogFileObject : public google::base::Logger {
 public:
  LogFileObject(google::LogSeverity severity, const char* base_filename);
  ~LogFileObject();

  void Write(bool force_flush, time_t timestamp, const char* message,
             int message_len) override;

  void SetBasename(const char* basename);
  void SetExtension(const char* ext);
  void SetSymlinkBasename(const char* symlink_basename);

  void Flush() override;

  // It is the actual file length for the system loggers
  virtual uint32_t LogSize() {
    std::lock_guard<std::mutex> lock(lock_);
    return file_length_;
  }

  void FlushUnlocked();

 private:
  bool CreateLogfile(const std::string& time_pid_string);
  static const uint32_t kRolloverAttemptFrequency = 0x20;
  std::mutex lock_;
  bool base_filename_selected_;
  std::string base_filename_;
  std::string base_filepath_;
  std::string symlink_basename_;
  std::string filename_extension_;
  FILE* file_;
  google::LogSeverity severity_;
  uint32_t bytes_since_flush_;
  uint32_t dropped_mem_length_;
  uint32_t file_length_;
  unsigned int rollover_attempt_;
  int64_t next_flush_time_;  // cycle count of which to flush log
};

}  // namespace logger
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_LOGGER_LOG_FILE_OBJECT_H_
