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
#include <algorithm>
#include <map>
#include <vector>
#include "modules/perception/tool/benchmark/lidar/base/frame.h"
#include "modules/perception/tool/benchmark/lidar/eval/meta_statistics.h"

namespace apollo {
namespace perception {
namespace benchmark {

struct ObjectMatch {
  unsigned int first;   // groundtruth object id
  unsigned int second;  // detected object id
  double jaccard_index;
  double jaccard_index_over_gt;
  unsigned int matched_point_num;
  double confidence;
};

class FrameStatistics : public Frame {
 public:
  bool find_association();
  bool cal_meta_statistics();
  const MetaStatistics& get_meta_statistics() const { return _meta_stat; }
  double jaccard_index_percentile() const;

 public:
  static void set_jaccard_index_threshold(double threshold);
  static void set_jaccard_index_percentile(double percentile);
  static double get_jaccard_index_percentile() {
    return _s_jaccard_index_percentile;
  }
  static void set_roi_is_main_lanes(bool value);

 private:
  // association data
  std::vector<unsigned int> _isolated_object_indices_2017;
  std::vector<unsigned int> _isolated_gt_object_indices_2017;
  std::vector<unsigned int> _underseg_gt_object_indices_2017;
  std::vector<unsigned int> _isolated_gt_object_indices_2016;
  std::vector<ObjectMatch> _matches;
  std::vector<unsigned int> _strict_match_indices;
  std::vector<PositionMetric> _object_position;
  std::vector<PositionMetric> _gt_object_position;
  std::vector<OrientationSimilarityMetric> _orientation_similarity;
  std::vector<double> _jaccard_indices;
  PositionMetricOption _position_option;
  // meta statistics data
  MetaStatistics _meta_stat;

 private:
  static double _s_jaccard_index_threshold;
  static double _s_jaccard_index_percentile;
  static double _s_cover_rate;
  static bool _s_roi_is_main_lanes;
};

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
