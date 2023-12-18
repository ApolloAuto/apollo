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

#include "modules/dreamview/backend/common/sim_control_manager/common/sim_control_util.h"

#include <cmath>

namespace apollo {
namespace dreamview {

double SimControlUtil::interpolate_1d(const double& p1, const double& p2,
                                      const double& frac1) {
  return p1 * (1.0 - frac1) + p2 * frac1;
}

double SimControlUtil::interpolated_find(const std::vector<double>& range_table,
                                         const std::vector<double>& val_table,
                                         double to_find) {
  int size = range_table.size();
  int left = -1;
  int right = size;
  int mid = 0;

  // assert range_table[right] >= to_find and range_table[left] < to_find
  while (left + 1 != right) {
    mid = ((right - left) >> 1) + left;

    if (range_table[mid] >= to_find) {
      right = mid;
    } else {
      left = mid;
    }
  }

  if (left == -1) {
    return val_table[0];
  }

  if (left == (size - 1)) {
    return val_table[range_table.size() - 1];
  }

  double range = range_table[right] - range_table[left];

  if (fabs(range) < 1e-6) {
    return 0.0;
  }

  double fraction = (to_find - range_table[left]) / range;

  return interpolate_1d(val_table[left], val_table[right], fraction);
}

double SimControlUtil::sigmoid(const double value) {
  return 1 / (1 + std::exp(-1.0 * value));
}

double SimControlUtil::relu(const double value) {
  return (value > 0.0) ? value : 0.0;
}

double SimControlUtil::normalize(const double value, const double mean,
                                 const double std) {
  double eps = 1e-10;
  return (value - mean) / (std + eps);
}

}  // namespace dreamview
}  // namespace apollo
