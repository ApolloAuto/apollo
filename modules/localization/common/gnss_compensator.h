/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "cyber/time/time.h"

namespace apollo {
namespace localization {

class LocalizationGnssCompensator {
 public:
  ~LocalizationGnssCompensator();

  /**
  * @brief compensate the time parsed from invalid GNSS data
  *
  * @param current_send_tf_time the time of sending tf msg
  * @param measurement_time GNSS data measurement time
  */
  void ProcessCompensation(
    const uint64_t& current_send_tf_time, uint64_t* measurement_time);

 private:
  uint64_t last_send_tf_time_ = 0;
  uint64_t last_valid_gnss_time_ = 0;
  uint64_t last_compensated_gnss_time_ = 0;

  DECLARE_SINGLETON(LocalizationGnssCompensator)
};

}  // namespace localization
}  // namespace apollo
