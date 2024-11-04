/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file message_manager.h
 * @brief The class of MessageManager
 */
#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

#include "modules/common_msgs/basic_msgs/error_code.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/canbus/common/byte.h"

/**
 * @namespace apollo::drivers::canbus
 * @brief apollo::drivers::canbus
 */
namespace apollo {
namespace drivers {
namespace canbus {

using apollo::common::ErrorCode;
using apollo::cyber::Time;
using micros = std::chrono::microseconds;

/**
 * @struct CheckIdArg
 *
 * @brief this struct include data for check ids.
 */
struct CheckIdArg {
  int64_t period = 0;
  int64_t real_period = 0;
  int64_t last_time = 0;
  int32_t error_count = 0;
};

/**
 * @class MessageManager
 *
 * @brief message manager manages protocols. It supports parse and can get
 * protocol data by message id.
 */
template <typename SensorType>
class MessageManager {
 public:
  /*
   * @brief constructor function
   */
  MessageManager() {}
  /*
   * @brief destructor function
   */
  virtual ~MessageManager() = default;

  /**
   * @brief parse data and store parsed info in receive protocol data
   * @param message_id the id of the message
   * @param data a pointer to the data array to be parsed
   * @param length the length of data array
   */
  virtual void Parse(const uint32_t message_id, const uint8_t *data,
                     int32_t length);

  /**
   * @brief parse data and store parsed info in send protocol data
   * @param message_id the id of the message
   * @param data a pointer to the data array to be parsed
   * @param length the length of data array
   */
  virtual void ParseSender(const uint32_t message_id, const uint8_t *data,
                           int32_t length);

  void ClearSensorData();
  void ClearSensorRecvData();
  void ClearSensorCheckRecvData();
  void ClearSensorSenderData();
  void ClearSensorCheckSenderData();

  std::condition_variable *GetMutableCVar();

  /**
   * @brief get mutable protocol data by message id
   * @param message_id the id of the message
   * @return a pointer to the protocol data
   */
  ProtocolData<SensorType> *GetMutableProtocolDataById(
      const uint32_t message_id);

  /**
   * @brief get chassis detail. used lock_guard in this function to avoid
   * concurrent read/write issue.
   * @param chassis_detail chassis_detail to be filled.
   */
  common::ErrorCode GetSensorData(SensorType *const sensor_data);

  /**
   * @brief get chassis recv detail. used lock_guard in this function to avoid
   * concurrent read/write issue.
   * @param chassis_detail chassis_detail to be filled.
   */
  common::ErrorCode GetSensorRecvData(SensorType *const sensor_recv_data);

  /**
   * @brief get chassis recv detail. used lock_guard in this function to avoid
   * concurrent read/write issue.
   * @param chassis_detail chassis_detail to be filled.
   */
  common::ErrorCode GetSensorCheckRecvData(SensorType *const sensor_recv_data);

  /**
   * @brief get chassis sender detail. used lock_guard in this function to avoid
   * concurrent read/write issue.
   * @param chassis_detail chassis_detail to be filled.
   */
  common::ErrorCode GetSensorSenderData(SensorType *const sensor_sender_data);

  /**
   * @brief get chassis sender detail. used lock_guard in this function to avoid
   * concurrent read/write issue.
   * @param chassis_detail chassis_detail to be filled.
   */
  common::ErrorCode GetSensorCheckSenderData(
      SensorType *const sensor_sender_data);

  /*
   * @brief reset send messages
   */
  void ResetSendMessages();

 protected:
  template <class T, bool need_check>
  void AddRecvProtocolData();

  template <class T, bool need_check>
  void AddSendProtocolData();

  std::vector<std::unique_ptr<ProtocolData<SensorType>>> send_protocol_data_;
  std::vector<std::unique_ptr<ProtocolData<SensorType>>> recv_protocol_data_;

  std::unordered_map<uint32_t, ProtocolData<SensorType> *> protocol_data_map_;
  std::unordered_map<uint32_t, ProtocolData<SensorType> *>
      recv_protocol_data_map_;
  std::unordered_map<uint32_t, ProtocolData<SensorType> *>
      sender_protocol_data_map_;

  std::unordered_map<uint32_t, CheckIdArg> check_ids_;
  std::set<uint32_t> received_ids_;

  std::mutex sensor_data_mutex_;
  SensorType sensor_data_;
  std::mutex sensor_data_recv_mutex_;
  SensorType sensor_recv_data_;
  std::mutex sensor_data_check_recv_mutex_;
  SensorType sensor_check_recv_data_;
  std::mutex sensor_data_sender_mutex_;
  SensorType sensor_sender_data_;
  std::mutex sensor_data_check_sender_mutex_;
  SensorType sensor_check_sender_data_;
  bool is_received_on_time_ = false;

  std::condition_variable cvar_;
};

template <typename SensorType>
template <class T, bool need_check>
void MessageManager<SensorType>::AddRecvProtocolData() {
  recv_protocol_data_.emplace_back(new T());
  auto *dt = recv_protocol_data_.back().get();
  if (dt == nullptr) {
    return;
  }
  recv_protocol_data_map_[T::ID] = dt;
  protocol_data_map_[T::ID] = dt;
  if (need_check) {
    check_ids_[T::ID].period = dt->GetPeriod();
    check_ids_[T::ID].real_period = 0;
    check_ids_[T::ID].last_time = 0;
    check_ids_[T::ID].error_count = 0;
  }
}

template <typename SensorType>
template <class T, bool need_check>
void MessageManager<SensorType>::AddSendProtocolData() {
  send_protocol_data_.emplace_back(new T());
  auto *dt = send_protocol_data_.back().get();
  if (dt == nullptr) {
    return;
  }
  sender_protocol_data_map_[T::ID] = dt;
  protocol_data_map_[T::ID] = dt;
  if (need_check) {
    check_ids_[T::ID].period = dt->GetPeriod();
    check_ids_[T::ID].real_period = 0;
    check_ids_[T::ID].last_time = 0;
    check_ids_[T::ID].error_count = 0;
  }
}

template <typename SensorType>
ProtocolData<SensorType> *
MessageManager<SensorType>::GetMutableProtocolDataById(
    const uint32_t message_id) {
  ADEBUG << "get protocol data message_id is:" << Byte::byte_to_hex(message_id);
  if (protocol_data_map_.find(message_id) == protocol_data_map_.end()) {
    ADEBUG << "Unable to get protocol data because of invalid message_id:"
           << Byte::byte_to_hex(message_id);
    return nullptr;
  }
  return protocol_data_map_[message_id];
}

template <typename SensorType>
void MessageManager<SensorType>::Parse(const uint32_t message_id,
                                       const uint8_t *data, int32_t length) {
  ProtocolData<SensorType> *protocol_data =
      GetMutableProtocolDataById(message_id);
  if (protocol_data == nullptr) {
    return;
  }
  // parse all
  {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    protocol_data->Parse(data, length, &sensor_data_);
  }

  // parse revceiver
  {
    std::lock_guard<std::mutex> lock(sensor_data_recv_mutex_);
    protocol_data->Parse(data, length, &sensor_recv_data_);
  }
  {
    std::lock_guard<std::mutex> lock(sensor_data_check_recv_mutex_);
    protocol_data->Parse(data, length, &sensor_check_recv_data_);
  }
  if (recv_protocol_data_map_.find(message_id) ==
      recv_protocol_data_map_.end()) {
    AERROR << "Failed to get recv data, " << "message is " << message_id;
  }

  received_ids_.insert(message_id);
  // check if need to check period
  const auto it = check_ids_.find(message_id);
  if (it != check_ids_.end()) {
    const int64_t time = Time::Now().ToNanosecond() / 1e3;
    it->second.real_period = time - it->second.last_time;
    // if period 1.5 large than base period, inc error_count
    const double period_multiplier = 1.5;
    if (static_cast<double>(it->second.real_period) >
        (static_cast<double>(it->second.period) * period_multiplier)) {
      it->second.error_count += 1;
    } else {
      it->second.error_count = 0;
    }
    it->second.last_time = time;
  }
}

template <typename SensorType>
void MessageManager<SensorType>::ParseSender(const uint32_t message_id,
                                             const uint8_t *data,
                                             int32_t length) {
  ProtocolData<SensorType> *protocol_data =
      GetMutableProtocolDataById(message_id);
  if (protocol_data == nullptr) {
    return;
  }

  // parse sender
  {
    std::lock_guard<std::mutex> lock(sensor_data_sender_mutex_);
    protocol_data->Parse(data, length, &sensor_sender_data_);
  }
  {
    std::lock_guard<std::mutex> lock(sensor_data_check_sender_mutex_);
    protocol_data->Parse(data, length, &sensor_check_sender_data_);
  }
  if (sender_protocol_data_map_.find(message_id) ==
      sender_protocol_data_map_.end()) {
    AERROR << "Failed to get prase sender data, " << "message is "
           << message_id;
  }

  received_ids_.insert(message_id);
  // check if need to check period
  const auto it = check_ids_.find(message_id);
  if (it != check_ids_.end()) {
    const int64_t time = Time::Now().ToNanosecond() / 1e3;
    it->second.real_period = time - it->second.last_time;
    // if period 1.5 large than base period, inc error_count
    const double period_multiplier = 1.5;
    if (static_cast<double>(it->second.real_period) >
        (static_cast<double>(it->second.period) * period_multiplier)) {
      it->second.error_count += 1;
    } else {
      it->second.error_count = 0;
    }
    it->second.last_time = time;
  }
}

template <typename SensorType>
void MessageManager<SensorType>::ClearSensorData() {
  std::lock_guard<std::mutex> lock(sensor_data_mutex_);
  sensor_data_.Clear();
}

template <typename SensorType>
void MessageManager<SensorType>::ClearSensorRecvData() {
  std::lock_guard<std::mutex> lock(sensor_data_recv_mutex_);
  sensor_recv_data_.Clear();
}

template <typename SensorType>
void MessageManager<SensorType>::ClearSensorCheckRecvData() {
  std::lock_guard<std::mutex> lock(sensor_data_check_recv_mutex_);
  sensor_check_recv_data_.Clear();
}

template <typename SensorType>
void MessageManager<SensorType>::ClearSensorSenderData() {
  std::lock_guard<std::mutex> lock(sensor_data_sender_mutex_);
  sensor_sender_data_.Clear();
}

template <typename SensorType>
void MessageManager<SensorType>::ClearSensorCheckSenderData() {
  std::lock_guard<std::mutex> lock(sensor_data_check_sender_mutex_);
  sensor_check_sender_data_.Clear();
}

template <typename SensorType>
std::condition_variable *MessageManager<SensorType>::GetMutableCVar() {
  return &cvar_;
}

template <typename SensorType>
ErrorCode MessageManager<SensorType>::GetSensorData(
    SensorType *const sensor_data) {
  if (sensor_data == nullptr) {
    AERROR << "Failed to get sensor_data due to nullptr.";
    return ErrorCode::CANBUS_ERROR;
  }
  std::lock_guard<std::mutex> lock(sensor_data_mutex_);
  sensor_data->CopyFrom(sensor_data_);
  return ErrorCode::OK;
}

template <typename SensorType>
ErrorCode MessageManager<SensorType>::GetSensorRecvData(
    SensorType *const sensor_recv_data) {
  if (sensor_recv_data == nullptr) {
    AERROR << "Failed to get receiver data due to nullptr.";
    return ErrorCode::CANBUS_ERROR;
  }
  std::lock_guard<std::mutex> lock(sensor_data_recv_mutex_);
  sensor_recv_data->CopyFrom(sensor_recv_data_);
  return ErrorCode::OK;
}

template <typename SensorType>
ErrorCode MessageManager<SensorType>::GetSensorCheckRecvData(
    SensorType *const sensor_recv_data) {
  if (sensor_recv_data == nullptr) {
    AERROR << "Failed to get receiver data due to nullptr.";
    return ErrorCode::CANBUS_ERROR;
  }
  std::lock_guard<std::mutex> lock(sensor_data_check_recv_mutex_);
  sensor_recv_data->CopyFrom(sensor_check_recv_data_);
  return ErrorCode::OK;
}

template <typename SensorType>
ErrorCode MessageManager<SensorType>::GetSensorSenderData(
    SensorType *const sensor_sender_data) {
  if (sensor_sender_data == nullptr) {
    AERROR << "Failed to get sender data due to nullptr.";
    return ErrorCode::CANBUS_ERROR;
  }
  std::lock_guard<std::mutex> lock(sensor_data_sender_mutex_);
  sensor_sender_data->CopyFrom(sensor_sender_data_);
  return ErrorCode::OK;
}

template <typename SensorType>
ErrorCode MessageManager<SensorType>::GetSensorCheckSenderData(
    SensorType *const sensor_sender_data) {
  if (sensor_sender_data == nullptr) {
    AERROR << "Failed to get sender data due to nullptr.";
    return ErrorCode::CANBUS_ERROR;
  }
  std::lock_guard<std::mutex> lock(sensor_data_check_sender_mutex_);
  sensor_sender_data->CopyFrom(sensor_check_sender_data_);
  return ErrorCode::OK;
}

template <typename SensorType>
void MessageManager<SensorType>::ResetSendMessages() {
  for (auto &protocol_data : send_protocol_data_) {
    if (protocol_data == nullptr) {
      AERROR << "Invalid protocol data.";
    } else {
      protocol_data->Reset();
    }
  }
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
