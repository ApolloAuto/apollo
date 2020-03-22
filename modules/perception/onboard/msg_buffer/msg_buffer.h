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
#pragma once

#include <boost/circular_buffer.hpp>
#include <cfloat>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/cyber.h"
#include "gflags/gflags.h"

namespace apollo {
namespace perception {
namespace onboard {

DECLARE_int32(obs_msg_buffer_size);
DECLARE_double(obs_buffer_match_precision);

template <class T>
class MsgBuffer {
 public:
  typedef std::shared_ptr<T const> ConstPtr;
  typedef std::pair<double, ConstPtr> ObjectPair;

 public:
  MsgBuffer() : buffer_queue_(FLAGS_obs_msg_buffer_size) {}
  ~MsgBuffer() = default;

  MsgBuffer(const MsgBuffer&) = delete;
  MsgBuffer operator=(const MsgBuffer&) = delete;

  void Init(const std::string& channel, const std::string& name);

  // get nearest message
  int LookupNearest(double timestamp, ConstPtr* msg);
  // get latest message
  int LookupLatest(ConstPtr* msg);
  // get messages in (timestamp-period, timestamp+period)
  int LookupPeriod(double timestamp, double period,
                   std::vector<ObjectPair>* msgs);

 private:
  void MsgCallback(const ConstPtr& msg);

 private:
  std::string node_name_;
  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Reader<T>> msg_subscriber_;
  std::mutex buffer_mutex_;

  bool init_ = false;
  boost::circular_buffer<ObjectPair> buffer_queue_;
};

template <class T>
void MsgBuffer<T>::Init(const std::string& channel, const std::string& name) {
  int index = static_cast<int>(name.find_last_of('/'));
  if (index != -1) {
    node_name_ = name.substr(index + 1) + "_subscriber";
  } else {
    node_name_ = name + "_subscriber";
  }
  node_.reset(apollo::cyber::CreateNode(node_name_).release());

  std::function<void(const ConstPtr&)> register_call =
      std::bind(&MsgBuffer<T>::MsgCallback, this, std::placeholders::_1);
  msg_subscriber_ = node_->CreateReader<T>(channel, register_call);

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  buffer_queue_.set_capacity(FLAGS_obs_msg_buffer_size);
  init_ = true;
}

template <class T>
void MsgBuffer<T>::MsgCallback(const ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  double timestamp = msg->measurement_time();
  buffer_queue_.push_back(std::make_pair(timestamp, msg));
}

template <class T>
int MsgBuffer<T>::LookupNearest(double timestamp, ConstPtr* msg) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (!init_) {
    AERROR << "msg buffer is uninitialized.";
    return false;
  }
  if (buffer_queue_.empty()) {
    AERROR << "msg buffer is empty.";
    return false;
  }
  if (buffer_queue_.front().first - FLAGS_obs_buffer_match_precision >
      timestamp) {
    AERROR << "Your timestamp (" << timestamp
           << ") is earlier than the oldest timestamp ("
           << buffer_queue_.front().first << ").";
    return false;
  }
  if (buffer_queue_.back().first + FLAGS_obs_buffer_match_precision <
      timestamp) {
    AERROR << "Your timestamp (" << timestamp
           << ") is newer than the latest timestamp ("
           << buffer_queue_.back().first << ").";
    return false;
  }

  // loop to find nearest
  double distance = DBL_MAX;
  int idx = static_cast<int>(buffer_queue_.size()) - 1;
  for (; idx >= 0; --idx) {
    double temp_distance = fabs(timestamp - buffer_queue_[idx].first);
    if (temp_distance >= distance) {
      break;
    }
    distance = temp_distance;
  }
  *msg = buffer_queue_[idx + 1].second;

  return true;
}

template <class T>
int MsgBuffer<T>::LookupLatest(ConstPtr* msg) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (!init_) {
    AERROR << "Message buffer is uninitialized.";
    return false;
  }
  if (buffer_queue_.empty()) {
    AERROR << "Message buffer is empty.";
    return false;
  }
  *msg = buffer_queue_.back().second;
  return true;
}

template <class T>
int MsgBuffer<T>::LookupPeriod(const double timestamp, const double period,
                               std::vector<ObjectPair>* msgs) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (!init_) {
    AERROR << "Message buffer is uninitialized.";
    return false;
  }
  if (buffer_queue_.empty()) {
    AERROR << "Message buffer is empty.";
    return false;
  }
  if (buffer_queue_.front().first - FLAGS_obs_buffer_match_precision >
      timestamp) {
    AERROR << "Your timestamp (" << timestamp << ") is earlier than the oldest "
           << "timestamp (" << buffer_queue_.front().first << ").";
    return false;
  }
  if (buffer_queue_.back().first + FLAGS_obs_buffer_match_precision <
      timestamp) {
    AERROR << "Your timestamp (" << timestamp << ") is newer than the latest "
           << "timestamp (" << buffer_queue_.back().first << ").";
    return false;
  }

  const double lower_timestamp = timestamp - period;
  const double upper_timestamp = timestamp + period;
  for (const auto& obj_pair : buffer_queue_) {
    if (obj_pair.first < lower_timestamp) {
      continue;
    }
    if (obj_pair.first > upper_timestamp) {
      break;
    }
    msgs->push_back(obj_pair);
  }

  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
