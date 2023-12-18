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
#include "modules/perception/common/base/polynomial.h"

namespace apollo {
namespace perception {
namespace base {

Polynomial::Polynomial() { coeff_[0] = 0.0; }
Polynomial::~Polynomial() {}

const std::map<uint32_t, double>& Polynomial::getCoeff() const {
  return coeff_;
}

double& Polynomial::operator[](const uint32_t& order) {
  initialized_ = false;
  return coeff_[order];
}

double Polynomial::operator()(const double& x) {
  if (!initialized_) {
    index_gap_.resize(coeff_.size() - 1, 0);
    auto it = coeff_.begin();
    auto it_previous = coeff_.begin();
    ++it;
    size_t idx = 0;
    for (; it != coeff_.end(); ++it, ++it_previous) {
      const uint32_t& order = it->first;
      const uint32_t& order_pre = it_previous->first;
      uint32_t gap = order - order_pre;
      index_gap_[idx] = gap;
      ++idx;
      power_cache_[gap] = 0.0;
    }
    initialized_ = true;
  }

  for (auto& item : power_cache_) {
    item.second = x;
    for (size_t i = 0; i < item.first - 1; ++i) {
      item.second *= x;
    }
  }

  auto r_it = coeff_.rbegin();
  double sum = r_it->second;
  auto it_gap = index_gap_.rbegin();
  while (it_gap != index_gap_.rend()) {
    ++r_it;
    sum = power_cache_[*it_gap] * sum + r_it->second;
    ++it_gap;
  }

  return sum;
}

std::ostream& operator<<(std::ostream& o, const Polynomial& p) {
  const std::map<uint32_t, double>& coeff = p.getCoeff();
  size_t i = 0;
  const size_t coeff_num = coeff.size();
  for (auto it = coeff.rbegin(); it != coeff.rend(); ++it) {
    const uint32_t order = it->first;
    const double c = it->second;

    o << "(" << c << ")*(t^" << order << ")";
    if (i < coeff_num - 1) {
      o << " + ";
    }
    ++i;
  }
  return o;
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
