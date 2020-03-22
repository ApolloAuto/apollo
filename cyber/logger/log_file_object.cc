// Copyright (c) 1999, Google Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
#include "cyber/logger/log_file_object.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <vector>

#include "cyber/logger/logger_util.h"
#include "glog/log_severity.h"

namespace apollo {
namespace cyber {
namespace logger {

#define PATH_SEPARATOR '/'

// Globally disable log writing (if disk is full)
static bool stop_writing = false;

const char* const LogSeverityNames[NUM_SEVERITIES] = {"INFO", "WARNING",
                                                      "ERROR", "FATAL"};

LogFileObject::LogFileObject(LogSeverity severity, const char* base_filename)
    : base_filename_selected_(base_filename != nullptr),
      base_filename_((base_filename != nullptr) ? base_filename : ""),
      symlink_basename_("UNKNOWN"),
      filename_extension_(),
      file_(NULL),
      severity_(severity),
      bytes_since_flush_(0),
      file_length_(0),
      rollover_attempt_(kRolloverAttemptFrequency - 1),
      next_flush_time_(0) {
  if (base_filename_.empty()) {
    base_filename_ = "UNKNOWN";
  }
  assert(severity >= 0);
  assert(severity < NUM_SEVERITIES);
}

LogFileObject::~LogFileObject() {
  std::lock_guard<std::mutex> lock(lock_);
  if (file_ != nullptr) {
    fclose(file_);
    file_ = nullptr;
  }
}

void LogFileObject::SetBasename(const char* basename) {
  std::lock_guard<std::mutex> lock(lock_);
  base_filename_selected_ = true;
  if (base_filename_ != basename) {
    // Get rid of old log file since we are changing names
    if (file_ != nullptr) {
      fclose(file_);
      file_ = nullptr;
      rollover_attempt_ = kRolloverAttemptFrequency - 1;
    }
    base_filename_ = basename;
  }
}

void LogFileObject::SetExtension(const char* ext) {
  std::lock_guard<std::mutex> lock(lock_);
  if (filename_extension_ != ext) {
    // Get rid of old log file since we are changing names
    if (file_ != nullptr) {
      fclose(file_);
      file_ = nullptr;
      rollover_attempt_ = kRolloverAttemptFrequency - 1;
    }
    filename_extension_ = ext;
  }
}

void LogFileObject::SetSymlinkBasename(const char* symlink_basename) {
  std::lock_guard<std::mutex> lock(lock_);
  symlink_basename_ = symlink_basename;
}

void LogFileObject::Flush() {
  std::lock_guard<std::mutex> lock(lock_);
  FlushUnlocked();
}

void LogFileObject::FlushUnlocked() {
  if (file_ != nullptr) {
    fflush(file_);
    bytes_since_flush_ = 0;
  }
  // Figure out when we are due for another flush.
  const int64 next =
      (FLAGS_logbufsecs * static_cast<int64>(1000000));  // in usec
  next_flush_time_ = CycleClock_Now() + UsecToCycles(next);
}

bool LogFileObject::CreateLogfile(const string& time_pid_string) {
  string string_filename =
      base_filename_ + filename_extension_ + time_pid_string;
  const char* filename = string_filename.c_str();
  int fd = open(filename, O_WRONLY | O_CREAT | O_EXCL, FLAGS_logfile_mode);
  if (fd == -1) {
    return false;
  }
  // Mark the file close-on-exec. We don't really care if this fails
  fcntl(fd, F_SETFD, FD_CLOEXEC);

  file_ = fdopen(fd, "a");  // Make a FILE*.
  if (file_ == nullptr) {   // Man, we're screwed!
    close(fd);
    unlink(filename);  // Erase the half-baked evidence: an unusable log file
    return false;
  }

  // We try to create a symlink called <program_name>.<severity>,
  // which is easier to use.  (Every time we create a new logfile,
  // we destroy the old symlink and create a new one, so it always
  // points to the latest logfile.)  If it fails, we're sad but it's
  // no error.
  if (!symlink_basename_.empty()) {
    // take directory from filename
    const char* slash = strrchr(filename, PATH_SEPARATOR);
    const string linkname =
        symlink_basename_ + '.' + LogSeverityNames[severity_];
    string linkpath;
    if (slash) {
      linkpath = string(filename, slash - filename + 1);  // get dirname
    }
    linkpath += linkname;
    unlink(linkpath.c_str());  // delete old one if it exists

    // We must have unistd.h.
    // Make the symlink be relative (in the same dir) so that if the
    // entire log directory gets relocated the link is still valid.
    const char* linkdest = slash ? (slash + 1) : filename;
    if (symlink(linkdest, linkpath.c_str()) != 0) {
      // silently ignore failures
      AINFO << "symlink failed.";
    }

    // Make an additional link to the log file in a place specified by
    // FLAGS_log_link, if indicated
    if (!FLAGS_log_link.empty()) {
      linkpath = FLAGS_log_link + "/" + linkname;
      unlink(linkpath.c_str());  // delete old one if it exists
      if (symlink(filename, linkpath.c_str()) != 0) {
        // silently ignore failures
      }
    }
  }

  return true;  // Everything worked
}

void LogFileObject::Write(bool force_flush, time_t timestamp,
                          const char* message, int message_len) {
  std::lock_guard<std::mutex> lock(lock_);

  // We don't log if the base_name_ is "" (which means "don't write")
  if (base_filename_selected_ && base_filename_.empty()) {
    return;
  }

  if (static_cast<int>(file_length_ >> 20) >= MaxLogSize() || PidHasChanged()) {
    if (file_ != nullptr) {
      fclose(file_);
    }
    file_ = nullptr;
    file_length_ = bytes_since_flush_ = 0;
    rollover_attempt_ = kRolloverAttemptFrequency - 1;
  }

  // If there's no destination file, make one before outputting
  if (file_ == nullptr) {
    // Try to rollover the log file every 32 log messages.  The only time
    // this could matter would be when we have trouble creating the log
    // file.  If that happens, we'll lose lots of log messages, of course!
    if (++rollover_attempt_ != kRolloverAttemptFrequency) {
      return;
    }
    rollover_attempt_ = 0;

    struct ::tm tm_time;
    localtime_r(&timestamp, &tm_time);

    // The logfile's filename will have the date/time & pid in it
    ostringstream time_pid_stream;
    time_pid_stream.fill('0');
    time_pid_stream << 1900 + tm_time.tm_year << setw(2) << 1 + tm_time.tm_mon
                    << setw(2) << tm_time.tm_mday << '-' << setw(2)
                    << tm_time.tm_hour << setw(2) << tm_time.tm_min << setw(2)
                    << tm_time.tm_sec << '.' << GetMainThreadPid();
    const string& time_pid_string = time_pid_stream.str();

    // base filename always selected.
    if (base_filename_selected_) {
      if (!CreateLogfile(time_pid_string)) {
        perror("Could not create log file");
        fprintf(stderr, "COULD NOT CREATE LOGFILE '%s'!\n",
                time_pid_string.c_str());
        return;
      }
    }

    // Write a header message into the log file
    ostringstream file_header_stream;
    file_header_stream.fill('0');
    file_header_stream << "Log file created at: " << 1900 + tm_time.tm_year
                       << '/' << setw(2) << 1 + tm_time.tm_mon << '/' << setw(2)
                       << tm_time.tm_mday << ' ' << setw(2) << tm_time.tm_hour
                       << ':' << setw(2) << tm_time.tm_min << ':' << setw(2)
                       << tm_time.tm_sec << '\n'
                       << "Running on machine: " << hostname() << '\n'
                       << "Log line format: [IWEF]mmdd hh:mm:ss.uuuuuu "
                       << "threadid file:line] msg" << '\n';
    const string& file_header_string = file_header_stream.str();

    const int header_len = static_cast<int>(file_header_string.size());
    if (file_ == nullptr) {
      return;
    }
    fwrite(file_header_string.data(), 1, header_len, file_);
    file_length_ += header_len;
    bytes_since_flush_ += header_len;
  }

  // Write to LOG file
  if (!stop_writing) {
    // fwrite() doesn't return an error when the disk is full, for
    // messages that are less than 4096 bytes. When the disk is full,
    // it returns the message length for messages that are less than
    // 4096 bytes. fwrite() returns 4096 for message lengths that are
    // greater than 4096, thereby indicating an error.
    errno = 0;
    fwrite(message, 1, message_len, file_);
    if (FLAGS_stop_logging_if_full_disk &&
        errno == ENOSPC) {  // disk full, stop writing to disk
      stop_writing = true;  // until the disk is
      return;
    } else {
      file_length_ += message_len;
      bytes_since_flush_ += message_len;
    }
  } else {
    if (CycleClock_Now() >= next_flush_time_) {
      stop_writing = false;  // check to see if disk has free space.
    }
    return;  // no need to flush
  }

  // See important msgs *now*.  Also, flush logs at least every 10^6 chars,
  // or every "FLAGS_logbufsecs" seconds.
  if (force_flush || (bytes_since_flush_ >= 1000000) ||
      (CycleClock_Now() >= next_flush_time_)) {
    FlushUnlocked();
  }
}

/* static */
const string& LogFileObject::hostname() {
  if (hostname_.empty()) {
    GetHostName(&hostname_);
    if (hostname_.empty()) {
      hostname_ = "(unknown)";
    }
  }
  return hostname_;
}

}  // namespace logger
}  // namespace cyber
}  // namespace apollo
