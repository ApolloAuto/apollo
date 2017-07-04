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

#ifndef MODULES_CANBUS_VEHILCE_PROTOCOL_DATA_H_
#define MODULES_CANBUS_VEHILCE_PROTOCOL_DATA_H_

#include "modules/canbus/common/canbus_consts.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
 * @class ProtocolData
 *
 * @brief This is the base class of protocol data.
 */
class ProtocolData {
 public:
  /**
   * @brief static function, used to calculate the checksum of input array.
   * @param input the pointer to the start position of input array
   * @param length the length of the input array
   * @return the value of checksum
   */
  static std::uint8_t CalculateCheckSum(const uint8_t* input,
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
   * @param car_status the parsed car_status
   */
  virtual void Parse(const uint8_t* bytes, int32_t length,
                     ChassisDetail* car_status) const;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param timestamp the timestamp of input data
   * @param car_status the parsed car_status
   */
  virtual void Parse(const uint8_t* bytes, int32_t length,
                     const struct timeval& timestamp,
                     ChassisDetail* car_status) const;

  /*
   * @brief update the data
   */
  virtual void UpdateData(uint8_t* data);

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

template <typename T>
T ProtocolData::BoundedValue(T lower, T upper, T val) {
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

}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICLE_PROTOCOL_DATA_H_
