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
#ifndef MODULES_CANBUS_VEHICLE_MESSAGE_MANAGER_H_
#define MODULES_CANBUS_VEHICLE_MESSAGE_MANAGER_H_

#include <memory>
#include <mutex>
#include <set>
#include <unordered_map>
#include <vector>

#include "modules/canbus/vehicle/protocol_data.h"
#include "modules/common/proto/error_code.pb.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

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
class MessageManager {
 public:
  /*
   * @brief destructor function
   */
  virtual ~MessageManager() = default;

  /**
   * @brief parse data and store parsed info in protocol data
   * @param message_id the id of the message
   * @param data a pointer to the data array to be parsed
   * @param length the length of data array
   * @param timestamp the timestamp of data
   */
  virtual void Parse(const uint32_t message_id, const uint8_t* data,
                     int32_t length, struct timeval timestamp);

  /**
   * @brief get mutable protocol data by message id
   * @param message_id the id of the message
   * @return a pointer to the protocol data
   */
  ProtocolData* GetMutableProtocolDataById(const uint32_t message_id);

  /**
   * @brief get chassis detail. used lock_guard in this function to avoid
   * concurrent read/write issue.
   * @param chassis_detail chassis_detail to be filled.
   */
  ::apollo::common::ErrorCode GetChassisDetail(
      ChassisDetail* const chassis_detail);

  /*
   * @brief reset send messages
   */
  void ResetSendMessages();

 protected:
  template <class T, bool need_check>
  void AddRecvProtocolData();

  template <class T, bool need_check>
  void AddSendProtocolData();

 private:
  std::vector<std::unique_ptr<ProtocolData>> send_protocol_data_;
  std::vector<std::unique_ptr<ProtocolData>> recv_protocol_data_;

  std::unordered_map<uint32_t, ProtocolData*> protocol_data_map_;
  std::unordered_map<uint32_t, CheckIdArg> check_ids_;
  std::set<uint32_t> received_ids_;

  std::mutex chassis_detail_mutex_;
  ChassisDetail chassis_detail_;
  bool is_received_on_time_ = false;
};

template <class T, bool need_check>
void MessageManager::AddRecvProtocolData() {
  recv_protocol_data_.emplace_back(new T());
  auto* dt = recv_protocol_data_.back().get();
  if (dt == nullptr) {
    return;
  }
  protocol_data_map_[T::ID] = dt;
  if (need_check) {
    check_ids_[T::ID].period = dt->GetPeriod();
    check_ids_[T::ID].real_period = 0;
    check_ids_[T::ID].last_time = 0;
    check_ids_[T::ID].error_count = 0;
  }
}

template <class T, bool need_check>
void MessageManager::AddSendProtocolData() {
  send_protocol_data_.emplace_back(new T());
  auto* dt = send_protocol_data_.back().get();
  if (dt == nullptr) {
    return;
  }
  protocol_data_map_[T::ID] = dt;
  if (need_check) {
    check_ids_[T::ID].period = dt->GetPeriod();
    check_ids_[T::ID].real_period = 0;
    check_ids_[T::ID].last_time = 0;
    check_ids_[T::ID].error_count = 0;
  }
}

}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICLE_MESSAGE_MANAGER_H_
