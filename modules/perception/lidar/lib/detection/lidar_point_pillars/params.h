/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <vector>

namespace apollo {
namespace perception {
namespace lidar {

class Params {
 public:
  static constexpr float kPillarXSize = 0.25f;
  static constexpr float kPillarYSize = 0.25f;
  static constexpr float kPillarZSize = 8.0f;
  static constexpr float kMinXRange = -100.0f;
  static constexpr float kMinYRange = -70.0f;
  static constexpr float kMinZRange = -5.0f;
  static constexpr float kMaxXRange = 100.0f;
  static constexpr float kMaxYRange = 70.0f;
  static constexpr float kMaxZRange = 3.0f;
  static constexpr int kNumClass = 9;
  static constexpr int kMaxNumPillars = 30000;
  static constexpr int kMaxNumPointsPerPillar = 60;
  static constexpr int kNumPointFeature = 5;  // x, y, z, i, delta of time
  static constexpr int kNumGatherPointFeature = 9;
  static constexpr int kNumAnchor = 200 * 140 * 8 + 220 * 140 * 8;
  static constexpr int kNumOutputBoxFeature = 7;
  static constexpr int kBatchSize = 1;
  static constexpr int kNumIndsForScan = 1024;
  static constexpr int kNumThreads = 64;
  static constexpr int kNumBoxCorners = 4;

  static std::vector<int> AnchorStrides() { return std::vector<int>{4, 2}; }

  static std::vector<int> NumAnchorSets() { return std::vector<int>{8, 8}; }

  static std::vector<std::vector<float>> AnchorDxSizes() {
    return std::vector<std::vector<float>>{
        std::vector<float>{2.94046906f, 1.95017717f, 2.73050468f, 2.4560939f},
        std::vector<float>{2.49008838f, 0.60058911f, 0.76279481f, 0.66344886f,
                           0.39694519f}};
  }

  static std::vector<std::vector<float>> AnchorDySizes() {
    return std::vector<std::vector<float>>{
        std::vector<float>{11.1885991, 4.60718145f, 6.38352896f, 6.73778078f},
        std::vector<float>{0.48578221f, 1.68452161f, 2.09973778f, 0.7256437f,
                           0.40359262f}};
  }

  static std::vector<std::vector<float>> AnchorDzSizes() {
    return std::vector<std::vector<float>>{
        std::vector<float>{3.47030982f, 1.72270761f, 3.13312415f, 2.73004906f},
        std::vector<float>{0.98297065f, 1.27192197f, 1.44403034f, 1.75748069f,
                           1.06232151f}};
  }

  static std::vector<std::vector<float>> AnchorZCoors() {
    return std::vector<std::vector<float>>{
        std::vector<float>{-0.0715754f, -0.93897414f, -0.08168083f,
                           -0.37937912f},
        std::vector<float>{-1.27247968f, -1.03743013f, -0.99194854f,
                           -0.73911038f, -1.27868911f}};
  }

  static std::vector<std::vector<int>> NumAnchorRo() {
    return std::vector<std::vector<int>>{std::vector<int>{2, 2, 2, 2},
                                         std::vector<int>{2, 2, 2, 1, 1}};
  }

  static std::vector<std::vector<float>> AnchorRo() {
    return std::vector<std::vector<float>>{
        std::vector<float>{0, M_PI / 2, 0, M_PI / 2, 0, M_PI / 2, 0, M_PI / 2},
        std::vector<float>{0, M_PI / 2, 0, M_PI / 2, 0, M_PI / 2, 0, 0}};
  }

 private:
  Params() = default;
  ~Params() = default;
};  // class Params

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
