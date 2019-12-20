/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "gtest/gtest.h"

#include "modules/perception/camera/lib/calibrator/common/histogram_estimator.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(HistogramEstimatorTest, histogram_estimator_test) {
  HistogramEstimator estimator;
  HistogramEstimatorParams params1;
  HistogramEstimatorParams params2;
  params1 = params2;
  estimator.Init();
  estimator.Init(nullptr);

  estimator.Init(&params1);
  estimator.Process();

  estimator.get_val_estimation();
  estimator.get_hist();
  estimator.get_val_cur();
  estimator.get_val_estimation();
  estimator.get_bin_value(0);

  estimator.Push(params1.data_ep - 1.0f);
  estimator.Push(params1.data_ep + 1.0f);
  estimator.Push(params1.data_sp - 1.0f);
  estimator.Push(params1.data_sp + 1.0f);
  estimator.Push((params1.data_sp + params1.data_ep) / 2.0f);
  estimator.Process();
  estimator.Clear();

  estimator.Push((params1.data_sp + params1.data_ep) / 2.0f);
  estimator.Process();
  estimator.Clear();

  for (int i = 0; i < 10; i++) {
    estimator.Push((params1.data_sp + params1.data_ep) / 2.0f);
  }
  estimator.Process();
  estimator.Clear();

  for (int i = 0; i < 100; i++) {
    estimator.Push((params1.data_sp + params1.data_ep) / 2.0f);
  }
  estimator.Process();
  estimator.Clear();

  for (int i = 0; i < 10; i++) {
    estimator.Push(params1.data_sp * static_cast<float>(i) / 100.0f +
                   params1.data_ep * (1.0f - static_cast<float>(i) / 100.0f));
  }
  estimator.Process();
  estimator.Clear();

  for (int i = 0; i < 10; i++) {
    estimator.Push(params1.data_ep * static_cast<float>(i) / 10.0f +
                   params1.data_sp * (1.0f - static_cast<float>(i) / 10.0f));
  }
  estimator.Process();
  estimator.Clear();

  for (int i = 0; i < 10; i++) {
    estimator.Push(params1.data_sp * static_cast<float>(i) / 10.0f +
                   params1.data_ep * (1.0f - static_cast<float>(i) / 10.0f));
  }
  for (int i = 0; i < 10; i++) {
    estimator.Push((params1.data_sp + params1.data_ep) / 2.0f);
  }
  estimator.Process();
  estimator.Clear();

  for (int i = 0; i < 10; i++) {
    estimator.Push(params1.data_ep * static_cast<float>(i) / 10.0f +
                   params1.data_sp * (1.0f - static_cast<float>(i) / 10.0f));
  }
  for (int i = 0; i < 10; i++) {
    estimator.Push((params1.data_sp + params1.data_ep) / 2.0f);
  }
  estimator.Process();
  params1.smooth_kernel_radius = params1.nr_bins_in_histogram;
  estimator.Init(&params1);
  estimator.Process();
  estimator.Clear();
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
