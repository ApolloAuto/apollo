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
 * @file sensor_message_manager.h
 * @brief The class of SensorMessageManager
 */
#ifndef MODULES_DRIVERS_SENSOR_MESSAGE_MANAGER_H_
#define MODULES_DRIVERS_SENSOR_MESSAGE_MANAGER_H_

#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

#include "modules/canbus/common/byte.h"
#include "modules/common/log.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/common/time/time.h"
#include "modules/drivers/sensor_protocol_data.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {

using Clock = ::apollo::common::time::Clock;
using micros = std::chrono::microseconds;
using ::apollo::common::ErrorCode;

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
 * @class SensorMessageManager
 *
 * @brief sensor message manager manages sensor protocols. It supports parse and
 * can get
 * sensor protocol data by message id.
 */
template <typename SensorType>
class SensorMessageManager {
 public:
  /*
   * @brief constructor function
   */
  SensorMessageManager() {}
  /*
   * @brief destructor function
   */
  virtual ~SensorMessageManager() = default;

  /**
   * @brief parse data and store parsed info in sensor protocol data
   * @param message_id the id of the message
   * @param data a pointer to the data array to be parsed
   * @param length the length of data array
   */
  virtual void Parse(const uint32_t message_id, const uint8_t *data,
                     int32_t length);

  /**
   * @brief get mutable sensor protocol data by message id
   * @param message_id the id of the message
   * @return a pointer to the sensor protocol data
   */
  SensorProtocolData<SensorType> *GetMutableSensorProtocolDataById(
      const uint32_t message_id);

  /**
   * @brief get sensor data. used lock_guard in this function to avoid
   * concurrent read/write issue.
   * @param sensor_data sensor_data to be filled.
   */
  ::apollo::common::ErrorCode GetSensorData(SensorType *const sensor_data);

 protected:
  template <class T, bool need_check>
  void AddRecvProtocolData();

 private:
  std::vector<std::unique_ptr<SensorProtocolData<SensorType>>>
      recv_sensor_protocol_data_;

  std::unordered_map<uint32_t, SensorProtocolData<SensorType> *>
      sensor_protocol_data_map_;
  std::unordered_map<uint32_t, CheckIdArg> check_ids_;
  std::set<uint32_t> received_ids_;

  std::mutex sensor_data_mutex_;
  SensorType sensor_data_;
  bool is_received_on_time_ = false;
};

template <typename SensorType>
template <class T, bool need_check>
void SensorMessageManager<SensorType>::AddRecvProtocolData() {
  recv_sensor_protocol_data_.emplace_back(new T());
  auto *dt = recv_sensor_protocol_data_.back().get();
  if (dt == nullptr) {
    return;
  }
  sensor_protocol_data_map_[T::ID] = dt;
  if (need_check) {
    check_ids_[T::ID].period = dt->GetPeriod();
    check_ids_[T::ID].real_period = 0;
    check_ids_[T::ID].last_time = 0;
    check_ids_[T::ID].error_count = 0;
  }
}

template <typename SensorType>
SensorProtocolData<SensorType>
    *SensorMessageManager<SensorType>::GetMutableSensorProtocolDataById(
        const uint32_t message_id) {
  if (sensor_protocol_data_map_.find(message_id) ==
      sensor_protocol_data_map_.end()) {
    ADEBUG << "Unable to get protocol data because of invalid message_id:"
           << message_id;
    return nullptr;
  }
  return sensor_protocol_data_map_[message_id];
}

template <typename SensorType>
void SensorMessageManager<SensorType>::Parse(const uint32_t message_id,
                                             const uint8_t *data,
                                             int32_t length) {
  SensorProtocolData<SensorType> *sensor_protocol_data =
      GetMutableSensorProtocolDataById(message_id);
  if (sensor_protocol_data == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(sensor_data_mutex_);
  sensor_protocol_data->Parse(data, length, &sensor_data_);

  received_ids_.insert(message_id);
  // check if need to check period
  const auto it = check_ids_.find(message_id);
  if (it != check_ids_.end()) {
    const int64_t time = apollo::common::time::AsInt64<micros>(Clock::Now());
    it->second.real_period = time - it->second.last_time;
    // if period 1.5 large than base period, inc error_count
    const double period_multiplier = 1.5;
    if (it->second.real_period > (it->second.period * period_multiplier)) {
      it->second.error_count += 1;
    } else {
      it->second.error_count = 0;
    }
    it->second.last_time = time;
  }
}

template <typename SensorType>
ErrorCode SensorMessageManager<SensorType>::GetSensorData(
    SensorType *const sensor_data) {
  if (sensor_data == nullptr) {
    // TODO(lizh): create a proper ERROR log which can tell mobileye and radar.
    AERROR << "Failed to get sensor_data due to nullptr.";
    return ErrorCode::CANBUS_ERROR;
  }
  std::lock_guard<std::mutex> lock(sensor_data_mutex_);
  sensor_data->CopyFrom(sensor_data_);
  return ErrorCode::OK;
}

}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_SENSOR_MESSAGE_MANAGER_H_
