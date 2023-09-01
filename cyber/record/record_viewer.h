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

#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"

namespace apollo {
namespace cyber {
namespace record {

/**
 * @brief The record viewer.
 */
class RecordViewer {
 public:
  using RecordReaderPtr = std::shared_ptr<RecordReader>;

  /**
   * @brief The constructor with single reader.
   *
   * @param reader
   * @param begin_time
   * @param end_time
   * @param channels
   */
  RecordViewer(const RecordReaderPtr& reader, uint64_t begin_time = 0,
               uint64_t end_time = std::numeric_limits<uint64_t>::max(),
               const std::set<std::string>& channels = {});

  /**
   * @brief The constructor with multiple readers.
   *
   * @param readers
   * @param begin_time
   * @param end_time
   * @param channels
   */
  RecordViewer(const std::vector<RecordReaderPtr>& readers,
               uint64_t begin_time = 0,
               uint64_t end_time = std::numeric_limits<uint64_t>::max(),
               const std::set<std::string>& channels = std::set<std::string>());

  /**
   * @brief Is this record reader is valid.
   *
   * @return True for valid, false for not.
   */
  bool IsValid() const;

  /**
   * @brief Get begin time.
   *
   * @return Begin time (nanoseconds).
   */
  uint64_t begin_time() const { return begin_time_; }

  /**
   * @brief Get end time.
   *
   * @return end time (nanoseconds).
   */
  uint64_t end_time() const { return end_time_; }

  /**
   * @brief Get channel list.
   *
   * @return List container with all channel name string.
   */
  std::set<std::string> GetChannelList() const { return channel_list_; }

  /**
   * @brief The iterator.
   */
  class Iterator : public std::iterator<std::input_iterator_tag, RecordMessage,
                                        int, RecordMessage*, RecordMessage&> {
   public:
    /**
     * @brief The constructor of iterator with viewer.
     *
     * @param viewer
     * @param end
     */
    explicit Iterator(RecordViewer* viewer, bool end = false);

    /**
     * @brief The default constructor of iterator.
     */
    Iterator() {}

    /**
     * @brief The default destructor of iterator.
     */
    virtual ~Iterator() {}

    /**
     * @brief Overloading operator ==.
     *
     * @param other
     *
     * @return The result.
     */
    bool operator==(Iterator const& other) const;

    /**
     * @brief Overloading operator !=.
     *
     * @param other
     *
     * @return The result.
     */
    bool operator!=(const Iterator& rhs) const;

    /**
     * @brief Overloading operator ++.
     *
     * @return The result.
     */
    Iterator& operator++();

    /**
     * @brief Overloading operator ->.
     *
     * @return The pointer.
     */
    pointer operator->();

    /**
     * @brief Overloading operator *.
     *
     * @return The reference.
     */
    reference operator*();

   private:
    bool end_ = false;
    uint64_t index_ = 0;
    RecordViewer* viewer_ = nullptr;
    value_type message_instance_;
  };

  /**
   * @brief Get the begin iterator.
   *
   * @return The begin iterator.
   */
  Iterator begin();

  /**
   * @brief Get the end iterator.
   *
   * @return The end iterator.
   */
  Iterator end();

  /**
   * @brief Get current iterator.
   *
   * @return The current iterator.
   */
  Iterator curr_itr();

  void set_curr_itr(const Iterator& curr_itr);

 private:
  friend class Iterator;

  void Init();
  void Reset();
  void UpdateTime();
  bool FillBuffer();
  bool Update(RecordMessage* message);

  uint64_t begin_time_ = 0;
  uint64_t end_time_ = std::numeric_limits<uint64_t>::max();
  // User defined channels
  std::set<std::string> channels_;
  // All channel in user defined readers
  std::set<std::string> channel_list_;
  std::vector<RecordReaderPtr> readers_;
  std::vector<bool> readers_finished_;

  uint64_t curr_begin_time_ = 0;
  std::multimap<uint64_t, std::shared_ptr<RecordMessage>> msg_buffer_;

  const uint64_t kStepTimeNanoSec = 1000000000UL;  // 1 second
  const std::size_t kBufferMinSize = 128;

  Iterator itr_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_RECORD_VIEWER_H_
