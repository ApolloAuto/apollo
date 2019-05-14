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
#include <memory>
#include <vector>

#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/common/cloud_mask.h"

namespace apollo {
namespace perception {
namespace lidar {

enum class SppClassType {
  OTHERS = 0,
  SMALLMOT = 1,
  BIGMOT = 2,
  CYCLIST = 3,
  PEDESTRIAN = 4,
  CONE = 5,
  MAX_TYPE = 6,
};

struct SppPoint {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float h = 0.f;

  SppPoint() = default;

  SppPoint(const base::PointF& point, float height) {
    x = point.x;
    y = point.y;
    z = point.z;
    h = height;
  }
};

struct SppCluster {
  // 3d points and ids
  std::vector<SppPoint> points;
  std::vector<uint32_t> point_ids;
  // 2d pixels
  std::vector<uint32_t> pixels;
  // class probabilities and type
  std::vector<float> class_prob;
  SppClassType type = SppClassType::OTHERS;
  float yaw = 0.f;
  float confidence = 1.f;
  float top_z = kDefaultTopZ;
  size_t points_in_roi = 0;

  SppCluster() {
    points.reserve(kDefaultReserveSize);
    point_ids.reserve(kDefaultReserveSize);
    pixels.reserve(kDefaultReserveSize);
    class_prob.reserve(static_cast<size_t>(SppClassType::MAX_TYPE));
  }

  inline void AddPointSample(const base::PointF& point, float height,
                             uint32_t point_id) {
    points.push_back(SppPoint(point, height));
    point_ids.push_back(point_id);
  }

  void SortPoint() {
    std::vector<int> indices(points.size(), 0);
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&](const int lhs, const int rhs) {
                return points[lhs].z < points[rhs].z;
              });
    std::vector<SppPoint> points_target(points.size());
    std::vector<uint32_t> point_ids_target(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      points_target[i] = points[indices[i]];
      point_ids_target[i] = point_ids[indices[i]];
    }
    points.swap(points_target);
    point_ids.swap(point_ids_target);
  }

  inline void clear() {
    points.clear();
    point_ids.clear();
    pixels.clear();
    class_prob.clear();
    type = SppClassType::OTHERS;
    yaw = 0.f;
    confidence = 1.f;
    top_z = kDefaultTopZ;
    points_in_roi = 0;
  }

  inline void RemovePoints(const CloudMask& mask) {
    size_t valid = 0;
    for (size_t i = 0; i < point_ids.size(); ++i) {
      if (mask[point_ids[i]]) {
        if (valid != i) {
          point_ids[valid] = point_ids[i];
          points[valid] = points[i];
        }
        ++valid;
      }
    }
    points.resize(valid);
    point_ids.resize(valid);
  }

  static const size_t kDefaultReserveSize = 1000;
  static constexpr float kDefaultTopZ = 50.f;
};

typedef std::shared_ptr<SppCluster> SppClusterPtr;
typedef std::shared_ptr<const SppCluster> SppClusterConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
