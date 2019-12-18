/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/filters/digital_filter_coefficients.h"

#include <cmath>
#include <vector>

namespace apollo {
namespace common {

void LpfCoefficients(const double ts, const double cutoff_freq,
                     std::vector<double> *denominators,
                     std::vector<double> *numerators) {
  denominators->clear();
  numerators->clear();
  denominators->reserve(3);
  numerators->reserve(3);

  double wa = 2.0 * M_PI * cutoff_freq;  // Analog frequency in rad/s
  double alpha = wa * ts / 2.0;          // tan(Wd/2), Wd is discrete frequency
  double alpha_sqr = alpha * alpha;
  double tmp_term = std::sqrt(2.0) * alpha + alpha_sqr;
  double gain = alpha_sqr / (1.0 + tmp_term);

  denominators->push_back(1.0);
  denominators->push_back(2.0 * (alpha_sqr - 1.0) / (1.0 + tmp_term));
  denominators->push_back((1.0 - std::sqrt(2.0) * alpha + alpha_sqr) /
                          (1.0 + tmp_term));

  numerators->push_back(gain);
  numerators->push_back(2.0 * gain);
  numerators->push_back(gain);
}

void LpFirstOrderCoefficients(const double ts, const double settling_time,
                              const double dead_time,
                              std::vector<double> *denominators,
                              std::vector<double> *numerators) {
  // sanity check
  if (ts <= 0.0 || settling_time < 0.0 || dead_time < 0.0) {
    AERROR << "time cannot be negative";
    return;
  }

  const size_t k_d = static_cast<size_t>(dead_time / ts);
  double a_term;

  denominators->clear();
  numerators->clear();
  denominators->reserve(2);
  numerators->reserve(k_d + 1);  // size depends on dead-time

  if (settling_time == 0.0) {
    a_term = 0.0;
  } else {
    a_term = exp(-1 * ts / settling_time);
  }

  denominators->push_back(1.0);
  denominators->push_back(-a_term);
  numerators->insert(numerators->end(), k_d, 0.0);
  numerators->push_back(1 - a_term);
}

}  // namespace common
}  // namespace apollo
