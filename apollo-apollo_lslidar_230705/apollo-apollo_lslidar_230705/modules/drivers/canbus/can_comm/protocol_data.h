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
 * @file
 * @brief The class of ProtocolData
 */

#pragma once

#include <cmath>
#include <numeric>

#include "cyber/common/log.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

/**
 * @namespace apollo::drivers::canbus
 * @brief apollo::drivers::canbus
 */
namespace apollo {
namespace drivers {
namespace canbus {

/**
 * @class ProtocolData
 *
 * @brief This is the base class of protocol data.
 */
template <typename SensorType>
class ProtocolData {
 public:
  /**
   * @brief static function, used to calculate the checksum of input array.
   * @param input the pointer to the start position of input array
   * @param length the length of the input array
   * @return the value of checksum
   */
  static std::uint8_t CalculateCheckSum(const uint8_t *input,
                                        const uint32_t length);
  /**
   * @brief construct protocol data.
   */
  ProtocolData() = default;

  /**
   * @brief destruct protocol data.
   */
  virtual ~ProtocolData() = default;

  /*
   * @brief get interval period for canbus messages
   * @return the interval period in us (1e-6s)
   */
  virtual uint32_t GetPeriod() const;

  /*
   * @brief get the length of protocol data. The length is usually 8.
   * @return the length of protocol data.
   */
  virtual int32_t GetLength() const;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param sensor_data the parsed sensor_data
   */
  virtual void Parse(const uint8_t *bytes, int32_t length,
                     SensorType *sensor_data) const;

  /*
   * @brief update the data
   */
  virtual void UpdateData(uint8_t *data);

  /*
   * @brief update the heartbeat data
   */
  virtual void UpdateData_Heartbeat(uint8_t *data);

  /*
   * @brief reset the protocol data
   */
  virtual void Reset();

  /*
   * @brief check if the value is in [lower, upper], if not , round it to bound
   */
  template <typename T>
  static T BoundedValue(T lower, T upper, T val);

 private:
  const int32_t data_length_ = CANBUS_MESSAGE_LENGTH;
};

template <typename SensorType>
template <typename T>
T ProtocolData<SensorType>::BoundedValue(T lower, T upper, T val) {
  if (lower > upper) {
    return val;
  }
  if (val < lower) {
    return lower;
  }
  if (val > upper) {
    return upper;
  }
  return val;
}

// (SUM(input))^0xFF
template <typename SensorType>
uint8_t ProtocolData<SensorType>::CalculateCheckSum(const uint8_t *input,
                                                    const uint32_t length) {
  return static_cast<uint8_t>(std::accumulate(input, input + length, 0) ^ 0xFF);
}

template <typename SensorType>
uint32_t ProtocolData<SensorType>::GetPeriod() const {
  const uint32_t CONST_PERIOD = 100 * 1000;
  return CONST_PERIOD;
}

template <typename SensorType>
int32_t ProtocolData<SensorType>::GetLength() const {
  return data_length_;
}

template <typename SensorType>
void ProtocolData<SensorType>::Parse(const uint8_t *bytes, int32_t length,
                                     SensorType *sensor_data) const {}

template <typename SensorType>
void ProtocolData<SensorType>::UpdateData(uint8_t * /*data*/) {}

template <typename SensorType>
void ProtocolData<SensorType>::UpdateData_Heartbeat(uint8_t * /*data*/) {}

template <typename SensorType>
void ProtocolData<SensorType>::Reset() {}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
