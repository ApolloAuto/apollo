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

#include "modules/perception/common/i_lib/core/i_blas.h"
#include "modules/perception/common/i_lib/core/i_rand.h"
#include "modules/perception/common/i_lib/core/i_struct.h"

#include <limits>

namespace apollo {
namespace perception {
namespace common {
// Compute the number of trials for Ransac. Number of trials is chosen
// sufficiently high to ensure with a probability "confidence" that at
// least one of the random samples  of data is free from outliers.
// "inlierprob" is the probability that any selected data point is an inlier
inline int IRansacTrials(int sample_size, double confidence,
                         double inlierprob) {
  return inlierprob > 0.0
             ? IRound(IDiv(ILog(1.0 - confidence),
                           ILog(1.0 - IPow(inlierprob, sample_size))))
             : std::numeric_limits<int>::max();
}

// Using Ransac to fit a model to data set which contains outliers
// The function needs 2n entries of scratch space in inliers
template <typename T, int l, int lp, int k, int s,
          void (*HypogenFunc)(const T* x, const T* xp, T* model),
          void (*CostFunc)(const T* model, const T* x, const T* xp, int n,
                           int* nr_liner, int* inliers, T* cost, T error_tol),
          void (*RefitFunc)(T* x, T* xp, int* inliers, T* model, int n,
                            int nr_liner)>
bool RobustBinaryFitRansac(T* x, T* xp, int n, T* model, int* consensus_size,
                           int* inliers, T error_tol,
                           bool re_est_model_w_inliers = false,
                           bool adaptive_trial_count = false,
                           double confidence = 0.99, double inlierprob = 0.5,
                           int min_nr_inliers = s,
                           bool random_shuffle_inputs = false) {
  const int kSize = s;
  const int kLength = l;
  const int kLp = lp;
  const int kKsize = k;
  int indices[kSize];
  T samples_x[kLength * kSize];
  T samples_xp[kLp * kSize];
  T tmp_model[kKsize];
  T cost = std::numeric_limits<T>::max();
  T best_cost = std::numeric_limits<T>::max();

  if (n < min_nr_inliers) {
    return false;
  }

  double actual_inlierprob = 0.0, tmp_inlierprob;
  int nr_trials = IRansacTrials(s, confidence, inlierprob);

  int nr_inliers = 0;
  int rseed = I_DEFAULT_SEED;
  int sample_count = 0;
  int i, idxl, idxlp, il, ilp;
  *consensus_size = 0;  // initialize the size of the consensus set to zero
  IZero(model, k);      // initialize the model with zeros

  if (random_shuffle_inputs) {
    IRandomizedShuffle(x, xp, n, l, lp, &rseed);
  }

  while (nr_trials > sample_count) {
    // generate random indices
    IRandomSample(indices, s, n, &rseed);
    // prepare data for model fitting
    for (i = 0; i < s; ++i) {
      idxl = indices[i] * l;
      idxlp = indices[i] * lp;
      il = i * l;
      ilp = i * lp;
      ICopy(x + idxl, samples_x + il, l);
      ICopy(xp + idxlp, samples_xp + ilp, lp);
    }

    // estimate model
    HypogenFunc(samples_x, samples_xp, tmp_model);

    // validate model
    CostFunc(tmp_model, x, xp, n, &nr_inliers, inliers + n, &cost, error_tol);
    if ((nr_inliers > *consensus_size) ||
        (nr_inliers == *consensus_size && cost < best_cost)) {
      *consensus_size = nr_inliers;
      best_cost = cost;
      ICopy(tmp_model, model, k);
      ICopy(inliers + n, inliers, *consensus_size);  // record inlier indices
      if (adaptive_trial_count) {
        tmp_inlierprob = IDiv(static_cast<double>(*consensus_size), n);
        if (tmp_inlierprob > actual_inlierprob) {
          actual_inlierprob = tmp_inlierprob;
          nr_trials = IRansacTrials(s, confidence, actual_inlierprob);
        }
      }
    }
    sample_count++;
  }
  bool succeeded = *consensus_size >= min_nr_inliers;

  if (succeeded && re_est_model_w_inliers && RefitFunc != nullptr) {
    RefitFunc(x, xp, inliers, model, n, *consensus_size);
  }
  return succeeded;
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
