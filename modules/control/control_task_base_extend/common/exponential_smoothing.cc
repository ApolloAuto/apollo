/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include "modules/control/control_task_base_extend/common/exponential_smoothing.h"

#include <algorithm>

namespace apollo {
namespace control {

double ExponentialSmoothing::exponential_smoothing(
        const double &current_value,
        const double &last_value,
        const double &alpha) {
    return alpha * current_value + (1.0 - alpha) * last_value;
}

}  // namespace control
}  // namespace apollo
