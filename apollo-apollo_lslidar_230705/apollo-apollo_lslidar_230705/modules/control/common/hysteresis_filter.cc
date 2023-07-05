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

#include "modules/control/common/hysteresis_filter.h"

namespace apollo {
namespace control {

void HysteresisFilter::filter(const double input_value, const double threshold,
                              const double hysteresis_upper,
                              const double hysteresis_lower, int *state,
                              double *output_value) {
  // Use integer to represent mode as of now, for instance,
  // 1 is throttle, 0 is brake, then threshold is speed error
  if (input_value > threshold + hysteresis_upper) {
    *state = 1;
    previous_state_ = *state;
    *output_value = threshold + hysteresis_upper;
  } else if (input_value < threshold - hysteresis_lower) {
    *state = 0;
    previous_state_ = *state;
    *output_value = threshold - hysteresis_lower;
  } else {
    *state = previous_state_;
    *output_value = input_value;
  }
}

}  // namespace control
}  // namespace apollo
