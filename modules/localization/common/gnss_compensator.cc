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

#include "modules/localization/common/gnss_compensator.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

LocalizationGnssCompensator::LocalizationGnssCompensator() {}

LocalizationGnssCompensator::~LocalizationGnssCompensator() {}

void LocalizationGnssCompensator::ProcessCompensation(
    const uint64_t& current_send_tf_time, uint64_t* measurement_time) {
  // Sometimes GNSS does not return valid timestamp
  // The following logic is to compensate the invalid timestamp of GNSS
  // we use 3 variables to make this compensation
  // last_valid_gnss_time_: Time parsed from last valid GNSS data
  // last_send_tf_time_: Time of the last tf sent
  // last_compensated_gnss_time_: Time of compensated gnss time
  uint64_t compensated_delta;
  if (last_valid_gnss_time_ != 0 && last_send_tf_time_ != 0) {
    // calculate delta
    compensated_delta = current_send_tf_time - last_send_tf_time_;
    // If exceed the tolerance range and the measurement time
    // is less than or equal to the last valid GNSS time, the
    // GNSS data is considered invalid.
    if (compensated_delta >=
        static_cast<uint64_t>(FLAGS_gps_imu_compensate_ns_tolerance) &&
        *measurement_time <= last_valid_gnss_time_) {
      // Record the time of compensation for use in case
      // the next GNSS data is also invalid
      last_compensated_gnss_time_ =
        last_compensated_gnss_time_+ compensated_delta;
      AINFO << "enter compensator: " << "last valid gnss time: " <<
        last_valid_gnss_time_ << ", " << "measurement time: " <<
        *measurement_time << ", " << "measurement time after compensated: " <<
        last_compensated_gnss_time_;
      // only compensate when the flag is true
      if (FLAGS_enable_gps_imu_compensate)
        *measurement_time = last_compensated_gnss_time_;
    } else {
      // If the GNSS data is valid, simply record it
      last_valid_gnss_time_ = *measurement_time;
      last_compensated_gnss_time_ = *measurement_time;
    }
  } else {
    // Initialize
    last_valid_gnss_time_ = *measurement_time;
    last_compensated_gnss_time_ = *measurement_time;
  }
  last_send_tf_time_ = current_send_tf_time;
}

}  // namespace localization
}  // namespace apollo
