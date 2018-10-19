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

#ifndef CYBER_RECORD_RECORD_VIEWER_H_
#define CYBER_RECORD_RECORD_VIEWER_H_

#include <memory>
#include <set>
#include <string>

#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"

namespace apollo {
namespace cyber {
namespace record {

class RecordMessage;

class RecordViewer {
 public:
  RecordViewer(const std::shared_ptr<RecordReader>& reader,
               uint64_t begin_time = 0, uint64_t end_time = UINT64_MAX,
               const std::set<std::string>& channels = std::set<std::string>());
  bool IsValid() const;
  bool Update(RecordMessage* message);
  uint64_t begin_time() const;
  uint64_t end_time() const;
  std::set<std::string> GetChannelList() const;

  class Iterator {
   public:
    using iterator_category = std::input_iterator_tag;
    using value_type = RecordMessage;
    using difference_type = int;
    using pointer = RecordMessage*;
    using reference = RecordMessage&;

    explicit Iterator(RecordViewer* viewer, bool end = false);
    Iterator() {}
    virtual ~Iterator() {}

    bool operator==(Iterator const& other) const;
    bool operator!=(const Iterator& rhs) const;
    void operator++();
    pointer operator->();
    reference operator*();

   private:
    bool end_ = false;
    uint64_t index_ = 0;
    RecordViewer* viewer_ = nullptr;
    value_type message_instance_;
  };

  Iterator begin();
  Iterator end();

 private:
  friend class Iterator;
  uint64_t begin_time_ = 0;
  uint64_t end_time_ = UINT64_MAX;
  std::set<std::string> channels_;
  std::shared_ptr<RecordReader> reader_ = nullptr;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_RECORD_READER_H_
