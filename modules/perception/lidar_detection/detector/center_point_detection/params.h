/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include <map>
#include <string>
#include <vector>

namespace apollo {
namespace perception {
namespace lidar {

class Params {
 public:
  static constexpr float kVoxelXSize = 0.2f;
  static constexpr float kVoxelYSize = 0.2f;
  static constexpr float kVoxelZSize = 8.0f;
  static constexpr float kMinXRange = -51.2f;
  static constexpr float kMinYRange = -51.2f;
  static constexpr float kMinZRange = -5.0f;
  static constexpr float kMaxXRange = 51.2f;
  static constexpr float kMaxYRange = 51.2f;
  static constexpr float kMaxZRange = 3.0f;
  static constexpr int kNumClass = 3;
  static constexpr int kMaxNumVoxels = 60000;
  static constexpr int kMaxNumPointsPerVoxel = 20;
  static constexpr int kNumPointFeature = 5;  // x, y, z, i, delta of time
  static constexpr int kNumOutputBoxFeature = 7;
  static constexpr int kBatchSize = 1;

 private:
  Params() = default;
  ~Params() = default;
};  // class Params

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
