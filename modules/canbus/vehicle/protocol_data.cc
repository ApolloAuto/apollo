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

#include <cmath>
#include "modules/canbus/vehicle/protocol_data.h"
#include "modules/common/log.h"

namespace apollo {
namespace canbus {

// (SUM(input))^0xFF
uint8_t ProtocolData::CalculateCheckSum(const uint8_t* input,
                                        const uint32_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; ++i) {
    sum += input[i];
  }
  return sum ^ 0xFF;
}

uint32_t ProtocolData::GetPeriod() const {
  const uint32_t CONST_PERIOD = 100 * 1000;
  return CONST_PERIOD;
}

int32_t ProtocolData::GetLength() const { return data_length_; }

void ProtocolData::Parse(const uint8_t* /*bytes*/, int32_t /*length*/,
                         ChassisDetail* /*car_status*/) const {}

void ProtocolData::Parse(const uint8_t* bytes, int32_t length,
                         const struct timeval& timestamp,
                         ChassisDetail* car_status) const {
  Parse(bytes, length, car_status);
}

void ProtocolData::UpdateData(uint8_t* /*data*/) {}

void ProtocolData::Reset() {}

}  // namespace canbus
}  // namespace apollo
