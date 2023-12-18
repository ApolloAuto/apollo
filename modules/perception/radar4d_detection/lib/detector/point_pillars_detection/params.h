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

#include <vector>

namespace apollo {
namespace perception {
namespace radar4d {

class Params {
 public:
  static constexpr float kPillarXSize = 0.16f;
  static constexpr float kPillarYSize = 0.16f;
  static constexpr float kPillarZSize = 5.0f;
  static constexpr float kMinXRange = 00.00f;
  static constexpr float kMinYRange = -25.60f;
  static constexpr float kMinZRange = -3.0f;
  static constexpr float kMaxXRange = 51.20f;
  static constexpr float kMaxYRange = 25.60f;
  static constexpr float kMaxZRange = 2.0f;
  static constexpr int kNumClass = 3;
  static constexpr int kMaxNumPillars = 16000;
  static constexpr int kMaxNumPointsPerPillar = 10;
  static constexpr int kNumPointFeature = 7;
  static constexpr int kNumAnchor = 160 * 160 * 6;
  static constexpr int kNumOutputBoxFeature = 7;
  static constexpr int kBatchSize = 1;
  static constexpr int kNumIndsForScan = 1024;
  static constexpr int kNumThreads = 64;
  static constexpr int kNumBoxCorners = 4;
  static constexpr int kGridSize = 160;
  static constexpr int kNumAnchorPerGrid = 6;

  static std::vector<int> AnchorStrides() { return std::vector<int>{2}; }

  static std::vector<int> NumAnchorSets() { return std::vector<int>{6}; }

  static std::vector<std::vector<float>> AnchorDxSizes() {
    return std::vector<std::vector<float>>{
        std::vector<float>{1.60, 0.60, 0.6}};
  }

  static std::vector<std::vector<float>> AnchorDySizes() {
    return std::vector<std::vector<float>>{
        std::vector<float>{3.90, 1.76, 0.8}};
  }

  static std::vector<std::vector<float>> AnchorDzSizes() {
    return std::vector<std::vector<float>>{
        std::vector<float>{1.56, 1.73, 1.73}};
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

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
