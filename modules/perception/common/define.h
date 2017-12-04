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

#ifndef MODULES_PERCEPTION_COMMON_DEFINE_H_
#define MODULES_PERCEPTION_COMMON_DEFINE_H_

namespace apollo {
namespace perception {

const double PI = 3.1415926535898;
const double kRadianToDegree = 57.29577951308232;

// Error code definition
enum StatusCode {
  SUCC = 0,
  // Common error, process will proceeding and warning log will be printed.
  // Under most circumstances, function should return this code when a error
  // occurs.
  FAIL = 1,
  // Fatal error, process will be terminated and fatal log will be printed.
  // Generated only when a fatal error occurs, such as config loading error.
  FATAL = 2,
  TAIL = 3,
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_COMMON_DEFINE_H_
