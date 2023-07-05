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

#include "cyber/record/file/record_file_reader.h"

#include "cyber/common/file.h"

namespace apollo {
namespace cyber {
namespace record {

using apollo::cyber::proto::SectionType;

bool RecordFileReader::Open(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  path_ = path;
  if (!::apollo::cyber::common::PathExists(path_)) {
    AERROR << "File not exist, file: " << path_;
    return false;
  }
  fd_ = open(path_.data(), O_RDONLY);
  if (fd_ < 0) {
    AERROR << "Open file failed, file: " << path_ << ", fd: " << fd_
           << ", errno: " << errno;
    return false;
  }
  end_of_file_ = false;
  if (!ReadHeader()) {
    AERROR << "Read header section fail, file: " << path_;
    return false;
  }
  return true;
}

void RecordFileReader::Close() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool RecordFileReader::Reset() {
  if (!SetPosition(sizeof(struct Section) + HEADER_LENGTH)) {
    AERROR << "Reset position fail, file: " << path_;
    return false;
  }
  end_of_file_ = false;
  return true;
}

bool RecordFileReader::ReadHeader() {
  Section section;
  if (!ReadSection(&section)) {
    AERROR << "Read header section fail, file is broken or it is not a record "
              "file.";
    return false;
  }
  if (section.type != SectionType::SECTION_HEADER) {
    AERROR << "Check section type failed"
           << ", expect: " << SectionType::SECTION_HEADER
           << ", actual: " << section.type;
    return false;
  }
  if (!ReadSection<proto::Header>(section.size, &header_)) {
    AERROR << "Read header section fail, file is broken or it is not a record "
              "file.";
    return false;
  }
  if (!SetPosition(sizeof(struct Section) + HEADER_LENGTH)) {
    AERROR << "Skip bytes for reaching the nex section failed.";
    return false;
  }
  return true;
}

bool RecordFileReader::ReadIndex() {
  if (!header_.is_complete()) {
    AERROR << "Record file is not complete.";
    return false;
  }
  if (!SetPosition(header_.index_position())) {
    AERROR << "Skip bytes for reaching the index section failed.";
    return false;
  }
  Section section;
  if (!ReadSection(&section)) {
    AERROR << "Read index section fail, maybe file is broken.";
    return false;
  }
  if (section.type != SectionType::SECTION_INDEX) {
    AERROR << "Check section type failed"
           << ", expect: " << SectionType::SECTION_INDEX
           << ", actual: " << section.type;
    return false;
  }
  if (!ReadSection<proto::Index>(section.size, &index_)) {
    AERROR << "Read index section fail.";
    return false;
  }
  Reset();
  return true;
}

bool RecordFileReader::ReadSection(Section* section) {
  ssize_t count = read(fd_, section, sizeof(struct Section));
  if (count < 0) {
    AERROR << "Read fd failed, fd_: " << fd_ << ", errno: " << errno;
    return false;
  } else if (count == 0) {
    end_of_file_ = true;
    AINFO << "Reach end of file.";
    return false;
  } else if (count != sizeof(struct Section)) {
    AERROR << "Read fd failed, fd_: " << fd_
           << ", expect count: " << sizeof(struct Section)
           << ", actual count: " << count;
    return false;
  }
  return true;
}

bool RecordFileReader::SkipSection(int64_t size) {
  int64_t pos = CurrentPosition();
  if (size > INT64_MAX - pos) {
    AERROR << "Current position plus skip count is larger than INT64_MAX, "
           << pos << " + " << size << " > " << INT64_MAX;
    return false;
  }
  if (!SetPosition(pos + size)) {
    AERROR << "Skip failed, file: " << path_ << ", current position: " << pos
           << "skip count: " << size;
    return false;
  }
  return true;
}

RecordFileReader::~RecordFileReader() {
  Close();
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
