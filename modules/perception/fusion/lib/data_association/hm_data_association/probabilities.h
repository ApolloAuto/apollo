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

#include <algorithm>
#include <vector>

namespace apollo {
namespace perception {
namespace fusion {

// @brief: bound given value within input range
// @return bounded value within input range
// @NOTE: original method name is bound
template <typename T>
T Bound(const T& value, const T& max_value, const T& min_value) {
  return std::max(min_value, std::min(max_value, value));
}

// @brief: scale input prob within input range
// @return bounded & scaled prob
// @NOTE: original method name is bound_scale_probability
double BoundedScalePositiveProbability(double p, double max_p, double min_p);

// @brief: scale input prob
// @return sclaed prob
// @NOTE: original method name is scale_positive_probability
double ScalePositiveProbability(double p, double max_p, double th_p);

// @brief: calculate the Welsh Loss
// @return Welsh Loss of input dist
// @NOTE: original method name is Welsh_var_loss_fun
double WelshVarLossFun(double dist, double th, double scale);

// @brief: fuse two probabilities, fused prob is greater than 0.5, if
// the sum of input prob pair is greater than 1, otherwise, fused prob
// is less than 0.5.
// @return fused prob of input prob pair
// @NOTE: original method name is fused_tow_probabilities
double FuseTwoProbabilities(double prob1, double prob2);

// @brief: fuse multiple probabilities
// @return fused probability of input multiple probabilities
// @NOTE: original method name is fused_multiple_probabilities
double FuseMultipleProbabilities(const std::vector<double>& probs);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
