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
  static constexpr float kPillarXSize = 0.32f;
  static constexpr float kPillarYSize = 0.32f;
  static constexpr float kPillarZSize = 6.0f;
  static constexpr float kMinXRange = -74.88f;
  static constexpr float kMinYRange = -74.88f;
  static constexpr float kMinZRange = -2.0f;
  static constexpr float kMaxXRange = 74.88f;
  static constexpr float kMaxYRange = 74.88f;
  static constexpr float kMaxZRange = 4.0f;
  static constexpr int kNumClass = 3;
  static constexpr int kMaxNumPillars = 32000;
  static constexpr int kMaxNumPointsPerPillar = 20;
  static constexpr int kNumPointFeature = 5;  // x, y, z, i, delta of time
  static constexpr int kNumAnchor = 468 * 468 * 6;
  static constexpr int kNumOutputBoxFeature = 7;
  static constexpr int kBatchSize = 1;
  static constexpr int kNumIndsForScan = 1024;
  static constexpr int kNumThreads = 64;
  static constexpr int kNumBoxCorners = 4;
  static constexpr int kGridSize = 468;
  static constexpr int kNumAnchorPerGrid = 6;

  static std::vector<int> AnchorStrides() { return std::vector<int>{1}; }

  static std::vector<int> NumAnchorSets() { return std::vector<int>{6}; }

  static std::vector<std::vector<float>> AnchorDxSizes() {
    return std::vector<std::vector<float>>{
        std::vector<float>{2.08, 0.84, 0.84}};
  }

  static std::vector<std::vector<float>> AnchorDySizes() {
    return std::vector<std::vector<float>>{
        std::vector<float>{4.73, 1.81, 0.91}};
  }

  static std::vector<std::vector<float>> AnchorDzSizes() {
    return std::vector<std::vector<float>>{
        std::vector<float>{1.77, 1.77, 1.74}};
  }

  static std::vector<std::vector<float>> AnchorZCoors() {
    return std::vector<std::vector<float>>{
        std::vector<float>{-0.0345, -0.1188, 0}};
  }

  static std::vector<std::vector<int>> NumAnchorRo() {
    return std::vector<std::vector<int>>{std::vector<int>{2, 2, 2}};
  }

  static std::vector<std::vector<float>> AnchorRo() {
    return std::vector<std::vector<float>>{
        std::vector<float>{0, M_PI / 2, 0, M_PI / 2, 0, M_PI / 2}};
  }

 private:
  Params() = default;
  ~Params() = default;
};  // class Params

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
