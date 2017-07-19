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
 * @file planning_error.h
 **/

#ifndef MODULES_PLANNING_COMMON_PLANNING_ERROR_H
#define MODULES_PLANNING_COMMON_PLANNING_ERROR_H

namespace apollo {
namespace planning {

enum class ErrorCode {
  PLANNING_SKIP = 1,
  PLANNING_OK = 0,
  PLANNING_ERROR_NOT_FOUND = -1,
  PLANNING_ERROR_OUT_OF_INDEX = -2,
  PLANNING_ERROR_SELF_LOOP = -3,
  PLANNING_ERROR_DUPLICATE = -4,
  PLANNING_ERROR_NULL_POINTER = -5,
  PLANNING_ERROR_NAN = -6,
  PLANNING_ERROR_TIMEOUT = -7,
  PLANNING_ERROR_FAILED = -8,
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PLANNING_ERROR_H
