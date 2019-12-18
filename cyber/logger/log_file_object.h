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
#include <iomanip>
#include <mutex>
#include <string>

#include "glog/logging.h"

namespace apollo {
namespace cyber {
namespace logger {

// the C99 format
typedef int32_t int32;
typedef uint32_t uint32;
typedef int64_t int64;
typedef uint64_t uint64;

using google::LogSeverity;
using google::NUM_SEVERITIES;
using std::ostringstream;
using std::setw;
using std::string;

// Encapsulates all file-system related state
class LogFileObject : public google::base::Logger {
 public:
  LogFileObject(LogSeverity severity, const char* base_filename);
  ~LogFileObject();

  void Write(bool force_flush,  // Should we force a flush here?
             time_t timestamp,  // Timestamp for this entry
             const char* message, int message_len) override;

  // Configuration options
  void SetBasename(const char* basename);
  void SetExtension(const char* ext);
  void SetSymlinkBasename(const char* symlink_basename);

  // Normal flushing routine
  void Flush() override;

  // It is the actual file length for the system loggers,
  // i.e., INFO, ERROR, etc.
  uint32 LogSize() override {
    std::lock_guard<std::mutex> lock(lock_);
    return file_length_;
  }

  // Internal flush routine.  Exposed so that FlushLogFilesUnsafe()
  // can avoid grabbing a lock.  Usually Flush() calls it after
  // acquiring lock_.
  void FlushUnlocked();

  const string& hostname();

 private:
  static const uint32 kRolloverAttemptFrequency = 0x20;

  std::mutex lock_;
  bool base_filename_selected_;
  string base_filename_;
  string symlink_basename_;
  string filename_extension_;  // option users can specify (eg to add port#)
  FILE* file_;
  LogSeverity severity_;
  uint32 bytes_since_flush_;
  uint32 file_length_;
  unsigned int rollover_attempt_;
  int64 next_flush_time_;  // cycle count at which to flush log
  string hostname_;

  // Actually create a logfile using the value of base_filename_ and the
  // supplied argument time_pid_string
  // REQUIRES: lock_ is held
  bool CreateLogfile(const string& time_pid_string);
};

}  // namespace logger
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_LOGGER_LOG_FILE_OBJECT_H_
