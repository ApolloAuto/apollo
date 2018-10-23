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

RecordFileReader::RecordFileReader() {}

RecordFileReader::~RecordFileReader() {}

bool RecordFileReader::Open(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  path_ = path;
  if (!::apollo::cyber::common::PathExists(path_)) {
    AERROR << "file not exist, file: " << path_;
    return false;
  }
  if (ifstream_.is_open()) {
    ifstream_.close();
  }
  std::ios_base::openmode mode = std::ios::binary | std::ios::in;
  ifstream_.open(path_, mode);
  if (!ifstream_ || !ifstream_.is_open()) {
    AERROR << "ifstream open fail, filename: " << path_ << "openmode: " << mode;
    return false;
  }
  return ReadHeader();
}

void RecordFileReader::Reset() {
  ifstream_.clear();
  ifstream_.seekg(0, std::ios::beg);
  ReadHeader();
}

bool RecordFileReader::ReadHeader() {
  uint64_t backup_position = ifstream_.tellg();
  ifstream_.seekg(0, std::ios::beg);
  Section section;
  if (!ReadSection(&section)) {
    AERROR << "read header section fail, file is broken of it is not a record "
              "file.";
    return false;
  }
  if (section.type != SectionType::SECTION_HEADER) {
    ifstream_.seekg(backup_position, std::ios::beg);
    AERROR << "this section is not header section, MUST BE a bug.";
    return false;
  }
  if (!ReadSection<Header>(section.size, &header_, HEADER_LENGTH)) {
    ifstream_.seekg(backup_position, std::ios::beg);
    AERROR << "read header section fail, file is broken of it is not a record "
              "file.";
    return false;
  }
  return true;
}

bool RecordFileReader::ReadIndex() {
  Section section;
  if (header_.index_position() <= sizeof(section) + HEADER_LENGTH) {
    AERROR << "index position in header is invalid, index position: "
           << header_.index_position();
    return false;
  }
  uint64_t backup_position = ifstream_.tellg();
  ifstream_.seekg(header_.index_position(), std::ios::beg);
  if (!ReadSection(&section)) {
    AERROR << "read index section fail, maybe file is broken."
              "file.";
    return false;
  }
  if (section.type != SectionType::SECTION_INDEX) {
    ifstream_.seekg(backup_position, std::ios::beg);
    AERROR << "this section is not index section, MUST BE a bug.";
    return false;
  }
  if (!ReadSection<Index>(section.size, &index_)) {
    ifstream_.seekg(backup_position, std::ios::beg);
    AERROR << "read index section fail, maybe file is broken.";
    return false;
  }
  ifstream_.seekg(backup_position, std::ios::beg);
  return true;
}

void RecordFileReader::Close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (ifstream_.is_open()) {
    ifstream_.close();
  }
}

bool RecordFileReader::ReadSection(Section* section) {
  if (!ifstream_.is_open()) {
    AERROR << "Input file stream is not open, file: " << path_;
    return false;
  }
  ifstream_.read(reinterpret_cast<char*>(section), sizeof(*section));
  if (ifstream_.eof()) {
    ADEBUG << "Failed to read section due to EOF, file: " << path_;
    return false;
  }
  if (ifstream_.gcount() != sizeof(*section)) {
    AERROR << "size of last readed is not equal with size of section"
           << " , file: " << path_ << " , actual size: " << ifstream_.gcount()
           << " , expect size: " << sizeof(*section);
    return false;
  }
  return true;
}

void RecordFileReader::SkipSection(uint64_t size, uint64_t fixed_size) {
  int64_t backup_position = ifstream_.tellg();
  if (size > 0) {
    ifstream_.seekg(backup_position + size, std::ios::beg);
  }
  if (fixed_size > size) {
    ifstream_.seekg(backup_position + fixed_size, std::ios::beg);
  }
}

bool RecordFileReader::EndOfFile() { return ifstream_.eof(); }

}  // namespace record
}  // namespace cyber
}  // namespace apollo
