/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <string>

namespace apollo {
namespace perception {
namespace lidar {

enum class LidarErrorCode {
  Succeed = 0,
  InitError = 1,
  PointCloudPreprocessorError = 2,
  MapManagerError = 3,
  SegmentationError = 4,
  ObjectBuilderError = 5,
  ObjectFilterError = 6,
  ClassifierError = 7,
  TrackerError = 8,
  DetectionError = 9,
};

struct LidarProcessResult {
  std::string log;
  LidarErrorCode error_code;

  explicit LidarProcessResult(const LidarErrorCode& error_code_in) {
    error_code = error_code_in;
  }

  LidarProcessResult(const LidarErrorCode& error_code_in,
                     const std::string& log_in) {
    error_code = error_code_in;
    log = log_in;
  }
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
