/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include <map>
#include <set>
#include <vector>
#include "modules/perception/tool/benchmark/lidar/util/geo_util.h"
#include "modules/perception/tool/benchmark/lidar/util/object.h"

namespace apollo {
namespace perception {
namespace benchmark {

enum class UpdateOperation { add = 0, remove = 1 };

class Visibility {
 public:
  explicit Visibility(float half_length, float half_width)
      : half_length_(half_length), half_width_(half_width) {}
  ~Visibility() = default;

  void set_car_pos(const Eigen::Vector3d& car_pos);

  void fill_objects(std::vector<ObjectPtr>* objs, float thresh);

 private:
  float calculate(std::vector<ObjectPtr>* objs, float thresh);

  void reset_state();

  void add_region();

  void add_segment(const VisPoint& a, const VisPoint& b, int idx);

  void query_segments(const VisPoint& p);

  void update_candidate_segment(const VisPoint& p, UpdateOperation op);

  float calculate_area(const VisPoint& a, const VisPoint& b);

  float calculate_visual_angle(const VisPoint& a, const VisPoint& b);

 private:
  VisPoint latest_query_point_;
  std::vector<Segment> latest_query_segments_cache_;
  std::multimap<VisPoint, Segment> points_;
  std::set<Segment> candidate_segment_;

  Eigen::Vector3d car_pos_;
  std::vector<float> full_visual_angle_;
  std::vector<float> visual_angle_;

  float half_length_ = 0;
  float half_width_ = 0;
  float area_sum_ = 0;
  static std::map<int, std::vector<size_t>> s_lut_;
};

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
