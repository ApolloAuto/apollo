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

#include "modules/common/math/aaboxkdtree2d.h"

#include <string>

#include "gtest/gtest.h"

#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace common {
namespace math {

namespace {

class Object {
 public:
  Object(const double x1, const double y1, const double x2, const double y2,
         const int id)
      : aabox_({x1, y1}, {x2, y2}),
        line_segment_({x1, y1}, {x2, y2}),
        id_(id) {}
  const AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const Vec2d &point) const {
    return line_segment_.DistanceTo(point);
  }
  double DistanceSquareTo(const Vec2d &point) const {
    return line_segment_.DistanceSquareTo(point);
  }
  int id() const { return id_; }

 private:
  AABox2d aabox_;
  LineSegment2d line_segment_;
  int id_ = 0;
};

}  // namespace

TEST(AABoxKDTree2dNode, OverallTests) {
  const int kNumBoxes[4] = {1, 10, 50, 100};
  const int kNumQueries = 1000;
  const double kSize = 100;
  const int kNumTrees = 4;
  AABoxKDTreeParams kdtree_params[kNumTrees];
  kdtree_params[1].max_depth = 2;
  kdtree_params[2].max_leaf_dimension = kSize / 4.0;
  kdtree_params[3].max_leaf_size = 20;

  for (int num_boxes : kNumBoxes) {
    std::vector<Object> objects;
    for (int i = 0; i < num_boxes; ++i) {
      const double cx = RandomDouble(-kSize, kSize);
      const double cy = RandomDouble(-kSize, kSize);
      const double dx = RandomDouble(-kSize / 10.0, kSize / 10.0);
      const double dy = RandomDouble(-kSize / 10.0, kSize / 10.0);
      objects.emplace_back(cx - dx, cy - dy, cx + dx, cy + dy, i);
    }
    std::unique_ptr<AABoxKDTree2d<Object>> kdtrees[kNumTrees];
    for (int i = 0; i < kNumTrees; ++i) {
      kdtrees[i].reset(new AABoxKDTree2d<Object>(objects, kdtree_params[i]));
    }
    for (int i = 0; i < kNumQueries; ++i) {
      const Vec2d point(RandomDouble(-kSize * 1.5, kSize * 1.5),
                        RandomDouble(-kSize * 1.5, kSize * 1.5));
      double expected_distance = std::numeric_limits<double>::infinity();
      for (const auto &object : objects) {
        expected_distance =
            std::min(expected_distance, object.DistanceTo(point));
      }
      for (int k = 0; k < kNumTrees; ++k) {
        const Object *nearest_object = kdtrees[k]->GetNearestObject(point);
        const double actual_distance = nearest_object->DistanceTo(point);
        EXPECT_NEAR(actual_distance, expected_distance, 1e-3);
      }
    }
    for (int i = 0; i < kNumQueries; ++i) {
      const Vec2d point(RandomDouble(-kSize * 1.5, kSize * 1.5),
                        RandomDouble(-kSize * 1.5, kSize * 1.5));
      const double distance = RandomDouble(0, kSize * 2.0);
      for (int k = 0; k < kNumTrees; ++k) {
        std::vector<const Object *> result_objects =
            kdtrees[k]->GetObjects(point, distance);
        std::set<int> result_ids;
        for (const Object *object : result_objects) {
          result_ids.insert(object->id());
        }
        EXPECT_EQ(result_objects.size(), result_ids.size());
        for (const auto &object : objects) {
          const double d = object.DistanceTo(point);
          if (std::abs(d - distance) <= 1e-3) {
            continue;
          }
          if (d < distance) {
            EXPECT_TRUE(result_ids.count(object.id()));
          } else {
            EXPECT_FALSE(result_ids.count(object.id()));
          }
        }
      }
    }
  }
}

}  // namespace math
}  // namespace common
}  // namespace apollo
