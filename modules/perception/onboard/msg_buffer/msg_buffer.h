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
#ifndef PERCEPTION_ONBOARD_MSG_BUFFER_H_
#define PERCEPTION_ONBOARD_MSG_BUFFER_H_

#include <boost/circular_buffer.hpp>
#include <string>
#include <utility>
#include <vector>

#include "gflags/gflags.h"

#include "cybertron/component/component.h"
#include "float.h"
#include "modules/perception/base/log.h"

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
  std::unique_ptr<cybertron::Node> node_;
  std::shared_ptr<cybertron::Reader<T>> msg_subscriber_;
  std::mutex buffer_mutex_;

  bool init_ = false;
  boost::circular_buffer<ObjectPair> buffer_queue_;
};

template <class T>
void MsgBuffer<T>::Init(const std::string& channel, const std::string& name) {
  int index = name.find_last_of('/');
  if (index != -1) {
    node_name_ = name.substr(index + 1) + "_subscriber";
  } else {
    node_name_ = name + "_subscriber";
  }
  node_.reset(new cybertron::Node(node_name_));

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
    return cybertron::FAIL;
  }
  if (buffer_queue_.empty()) {
    AERROR << "msg buffer is empty.";
    return cybertron::FAIL;
  }
  if (buffer_queue_.front().first - FLAGS_obs_buffer_match_precision >
      timestamp) {
    AERROR << "Your timestamp (" << GLOG_TIMESTAMP(timestamp)
           << ") is earlier than the oldest "
           << "timestamp (" << GLOG_TIMESTAMP(buffer_queue_.front().first)
           << ").";
    return cybertron::FAIL;
  }
  if (buffer_queue_.back().first + FLAGS_obs_buffer_match_precision <
      timestamp) {
    AERROR << "Your timestamp (" << GLOG_TIMESTAMP(timestamp)
           << ") is newer than the latest "
           << "timestamp (" << GLOG_TIMESTAMP(buffer_queue_.back().first)
           << ").";
    return cybertron::FAIL;
  }

  // loop to find nearest
  double distance = DBL_MAX;
  int idx = buffer_queue_.size() - 1;
  for (; idx >= 0; --idx) {
    double temp_distance = fabs(timestamp - buffer_queue_[idx].first);
    if (temp_distance >= distance) {
      break;
    }
    distance = temp_distance;
  }
  *msg = buffer_queue_[idx + 1].second;

  return cybertron::SUCC;
}

template <class T>
int MsgBuffer<T>::LookupLatest(ConstPtr* msg) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (!init_) {
    AERROR << "msg buffer is uninitialized.";
    return cybertron::FAIL;
  }
  if (buffer_queue_.empty()) {
    AERROR << "msg buffer is empty.";
    return cybertron::FAIL;
  }
  *msg = buffer_queue_.back().second;
  return cybertron::SUCC;
}

template <class T>
int MsgBuffer<T>::LookupPeriod(const double timestamp, const double period,
                               std::vector<ObjectPair>* msgs) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (!init_) {
    AERROR << "msg buffer is uninitialized.";
    return cybertron::FAIL;
  }
  if (buffer_queue_.empty()) {
    AERROR << "msg buffer is empty.";
    return cybertron::FAIL;
  }
  if (buffer_queue_.front().first - FLAGS_obs_buffer_match_precision >
      timestamp) {
    AERROR << "Your timestamp (" << GLOG_TIMESTAMP(timestamp)
           << ") is earlier than the oldest "
           << "timestamp (" << GLOG_TIMESTAMP(buffer_queue_.front().first)
           << ").";
    return cybertron::FAIL;
  }
  if (buffer_queue_.back().first + FLAGS_obs_buffer_match_precision <
      timestamp) {
    AERROR << "Your timestamp (" << GLOG_TIMESTAMP(timestamp)
           << ") is newer than the latest "
           << "timestamp (" << GLOG_TIMESTAMP(buffer_queue_.back().first)
           << ").";
    return cybertron::FAIL;
  }

  const double lower_timestamp = timestamp - period;
  const double upper_timestamp = timestamp + period;
  for (size_t idx = 0; idx < buffer_queue_.size(); ++idx) {
    if (buffer_queue_[idx].first < lower_timestamp) {
      continue;
    } else {
      if (buffer_queue_[idx].first > upper_timestamp) {
        break;
      } else {
        msgs->emplace_back(std::make_pair(buffer_queue_[idx].first,
                                          buffer_queue_[idx].second));
      }
    }
  }

  return cybertron::SUCC;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_ONBOARD_MSG_BUFFER_H_
