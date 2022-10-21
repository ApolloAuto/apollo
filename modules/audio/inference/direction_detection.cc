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

#include "modules/audio/inference/direction_detection.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace audio {

using torch::indexing::None;
using torch::indexing::Slice;
using apollo::common::math::NormalizeAngle;

DirectionDetection::DirectionDetection() {}

DirectionDetection::~DirectionDetection() {}

std::pair<Point3D, double> DirectionDetection::EstimateSoundSource(
    std::vector<std::vector<double>>&& channels_vec,
    const std::string& respeaker_extrinsic_file, const int sample_rate,
    const double mic_distance) {
  if (!respeaker2imu_ptr_.get()) {
    respeaker2imu_ptr_.reset(new Eigen::Matrix4d);
    LoadExtrinsics(respeaker_extrinsic_file, respeaker2imu_ptr_.get());
  }
  double degree =
      EstimateDirection(move(channels_vec), sample_rate, mic_distance);
  Eigen::Vector4d source_position(kDistance * sin(degree),
                                  kDistance * cos(degree), 0, 1);
  source_position = (*respeaker2imu_ptr_) * source_position;

  Point3D source_position_p3d;
  source_position_p3d.set_x(source_position[0]);
  source_position_p3d.set_y(source_position[1]);
  source_position_p3d.set_z(source_position[2]);
  degree = NormalizeAngle(degree);
  return {source_position_p3d, degree};
}

double DirectionDetection::EstimateDirection(
    std::vector<std::vector<double>>&& channels_vec, const int sample_rate,
    const double mic_distance) {
  std::vector<torch::Tensor> channels_ts;
  auto options = torch::TensorOptions().dtype(torch::kFloat64);
  int size = static_cast<int>(channels_vec[0].size());
  for (auto& signal : channels_vec) {
    channels_ts.push_back(torch::from_blob(signal.data(), {size}, options));
  }

  double tau0, tau1;
  double theta0, theta1;
  const double max_tau = mic_distance / kSoundSpeed;
  tau0 = GccPhat(channels_ts[0], channels_ts[2], sample_rate, max_tau, 1);
  theta0 = asin(tau0 / max_tau) * 180 / M_PI;
  tau1 = GccPhat(channels_ts[1], channels_ts[3], sample_rate, max_tau, 1);
  theta1 = asin(tau1 / max_tau) * 180 / M_PI;

  int best_guess = 0;
  if (fabs(theta0) < fabs(theta1)) {
    best_guess = theta1 > 0 ? std::fmod(theta0 + 360, 360) : (180 - theta0);
  } else {
    best_guess = theta0 < 0 ? std::fmod(theta1 + 360, 360) : (180 - theta1);
    best_guess = (best_guess + 90 + 180) % 360;
  }
  best_guess = (-best_guess + 480) % 360;

  return static_cast<double>(best_guess) / 180 * M_PI;
}

bool DirectionDetection::LoadExtrinsics(const std::string& yaml_file,
                                        Eigen::Matrix4d* respeaker_extrinsic) {
  if (!apollo::cyber::common::PathExists(yaml_file)) {
    AINFO << yaml_file << " does not exist!";
    return false;
  }
  YAML::Node node = YAML::LoadFile(yaml_file);
  double qw = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  try {
    if (node.IsNull()) {
      AINFO << "Load " << yaml_file << " failed! please check!";
      return false;
    }
    qw = node["transform"]["rotation"]["w"].as<double>();
    qx = node["transform"]["rotation"]["x"].as<double>();
    qy = node["transform"]["rotation"]["y"].as<double>();
    qz = node["transform"]["rotation"]["z"].as<double>();
    tx = node["transform"]["translation"]["x"].as<double>();
    ty = node["transform"]["translation"]["y"].as<double>();
    tz = node["transform"]["translation"]["z"].as<double>();
  } catch (YAML::Exception& e) {
    AERROR << "load camera extrinsic file " << yaml_file
           << " with error, YAML exception:" << e.what();
    return false;
  }
  respeaker_extrinsic->setConstant(0);
  Eigen::Quaterniond q;
  q.x() = qx;
  q.y() = qy;
  q.z() = qz;
  q.w() = qw;
  (*respeaker_extrinsic).block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  (*respeaker_extrinsic)(0, 3) = tx;
  (*respeaker_extrinsic)(1, 3) = ty;
  (*respeaker_extrinsic)(2, 3) = tz;
  (*respeaker_extrinsic)(3, 3) = 1;
  return true;
}

double DirectionDetection::GccPhat(const torch::Tensor& sig,
                                   const torch::Tensor& refsig, int fs,
                                   double max_tau, int interp) {
  const int n_sig = sig.size(0), n_refsig = refsig.size(0),
            n = n_sig + n_refsig;
  torch::Tensor psig = at::constant_pad_nd(sig, {0, n_refsig}, 0);
  torch::Tensor prefsig = at::constant_pad_nd(refsig, {0, n_sig}, 0);
  psig = at::view_as_real(at::fft_rfft(psig, at::nullopt, 1));
  prefsig = at::view_as_real(at::fft_rfft(prefsig, at::nullopt, 1));
  ConjugateTensor(&prefsig);
  torch::Tensor r = ComplexMultiply(psig, prefsig);
    torch::Tensor cc =
      at::fft_ifftn(at::view_as_complex(r / ComplexAbsolute(r)), {interp * n});
  int max_shift = static_cast<int>(interp * n / 2);
  if (max_tau != 0)
    max_shift = std::min(static_cast<int>(interp * fs * max_tau), max_shift);

  auto begin = cc.index({Slice(cc.size(0) - max_shift, None)});
  auto end = cc.index({Slice(None, max_shift + 1)});
  cc = at::cat({begin, end});
  // find max cross correlation index
  const int shift = at::argmax(at::abs(cc), 0).item<int>() - max_shift;
  const double tau = shift / static_cast<double>(interp * fs);

  return tau;
}

void DirectionDetection::ConjugateTensor(torch::Tensor* tensor) {
  tensor->index_put_({"...", 1}, -tensor->index({"...", 1}));
}

torch::Tensor DirectionDetection::ComplexMultiply(const torch::Tensor& a,
                                                  const torch::Tensor& b) {
  torch::Tensor real = a.index({"...", 0}) * b.index({"...", 0}) -
                       a.index({"...", 1}) * b.index({"...", 1});
  torch::Tensor imag = a.index({"...", 0}) * b.index({"...", 1}) +
                       a.index({"...", 1}) * b.index({"...", 0});
  return at::cat({real.reshape({-1, 1}), imag.reshape({-1, 1})}, 1);
}

torch::Tensor DirectionDetection::ComplexAbsolute(const torch::Tensor& tensor) {
  torch::Tensor res = tensor * tensor;
  res = at::sqrt(res.sum(1)).reshape({-1, 1});

  return res;
}

}  // namespace audio
}  // namespace apollo
