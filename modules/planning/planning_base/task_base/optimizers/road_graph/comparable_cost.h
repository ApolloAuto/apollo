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

/**
 * @file
 **/

#pragma once

namespace apollo {
namespace planning {

class ComparableCost {
 public:
  ComparableCost() = default;
  ComparableCost(const bool has_collision, const bool out_of_boundary,
                 const bool out_of_lane, const double safety_cost_,
                 const double smoothness_cost_)
      : safety_cost(safety_cost_), smoothness_cost(smoothness_cost_) {
    cost_items[HAS_COLLISION] = has_collision;
    cost_items[OUT_OF_BOUNDARY] = out_of_boundary;
    cost_items[OUT_OF_LANE] = out_of_lane;
  }
  ComparableCost(const ComparableCost &) = default;

  int CompareTo(const ComparableCost &other) const {
    for (size_t i = 0; i < cost_items.size(); ++i) {
      if (cost_items[i]) {
        if (other.cost_items[i]) {
          continue;
        } else {
          return 1;
        }
      } else {
        if (other.cost_items[i]) {
          return -1;
        } else {
          continue;
        }
      }
    }

    static constexpr double kEpsilon = 1e-12;
    const double diff = safety_cost + smoothness_cost - other.safety_cost -
                        other.smoothness_cost;
    if (std::fabs(diff) < kEpsilon) {
      return 0;
    } else if (diff > 0) {
      return 1;
    } else {
      return -1;
    }
  }
  ComparableCost operator+(const ComparableCost &other) {
    ComparableCost lhs = *this;
    lhs += other;
    return lhs;
  }
  ComparableCost &operator+=(const ComparableCost &other) {
    for (size_t i = 0; i < cost_items.size(); ++i) {
      cost_items[i] = (cost_items[i] || other.cost_items[i]);
    }
    safety_cost += other.safety_cost;
    smoothness_cost += other.smoothness_cost;
    return *this;
  }
  bool operator>(const ComparableCost &other) const {
    return this->CompareTo(other) > 0;
  }
  bool operator>=(const ComparableCost &other) const {
    return this->CompareTo(other) >= 0;
  }
  bool operator<(const ComparableCost &other) const {
    return this->CompareTo(other) < 0;
  }
  bool operator<=(const ComparableCost &other) const {
    return this->CompareTo(other) <= 0;
  }
  /*
   * cost_items represents an array of factors that affect the cost,
   * The level is from most critical to less critical.
   * It includes:
   * (0) has_collision or out_of_boundary
   * (1) out_of_lane
   *
   * NOTICE: Items could have same critical levels
   */
  static const size_t HAS_COLLISION = 0;
  static const size_t OUT_OF_BOUNDARY = 1;
  static const size_t OUT_OF_LANE = 2;
  std::array<bool, 3> cost_items = {{false, false, false}};

  // cost from distance to obstacles or boundaries
  double safety_cost = 0.0f;
  // cost from deviation from lane center, path curvature etc
  double smoothness_cost = 0.0f;
};

}  // namespace planning
}  // namespace apollo
