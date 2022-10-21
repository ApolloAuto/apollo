
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

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Eigen"
// Eigen 3.3.7: #define ALIVE (0)
// fastrtps: enum ChangeKind_t { ALIVE, ... };
#if defined(ALIVE)
#undef ALIVE
#endif

#include "ATen/ATen.h"
#include "torch/torch.h"

#include "modules/common_msgs/basic_msgs/geometry.pb.h"

namespace apollo {
namespace audio {

using apollo::common::Point3D;

class DirectionDetection {
 public:
  DirectionDetection();
  ~DirectionDetection();
  // Estimates the position of the source of the sound
  std::pair<Point3D, double> EstimateSoundSource(
      std::vector<std::vector<double>>&& channels_vec,
      const std::string& respeaker_extrinsic_file,
      const int sample_rate, const double mic_distance);

 private:
  const double kSoundSpeed = 343.2;
  const int kDistance = 50;
  std::unique_ptr<Eigen::Matrix4d> respeaker2imu_ptr_;

  // Estimates the direction of the source of the sound
  double EstimateDirection(std::vector<std::vector<double>>&& channels_vec,
                           const int sample_rate, const double mic_distance);

  bool LoadExtrinsics(const std::string& yaml_file,
                      Eigen::Matrix4d* respeaker_extrinsic);

  // Computes the offset between the signal sig and the reference signal refsig
  // using the Generalized Cross Correlation - Phase Transform (GCC-PHAT)method.
  double GccPhat(const torch::Tensor& sig, const torch::Tensor& refsig, int fs,
                 double max_tau, int interp);

  // Libtorch does not support Complex type currently.
  void ConjugateTensor(torch::Tensor* tensor);
  torch::Tensor ComplexMultiply(const torch::Tensor& a, const torch::Tensor& b);
  torch::Tensor ComplexAbsolute(const torch::Tensor& tensor);
};

}  // namespace audio
}  // namespace apollo
