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
#include "modules/perception/tool/benchmark/lidar/eval/frame_statistics.h"
#include <unistd.h>

namespace apollo {
namespace perception {
namespace benchmark {

double FrameStatistics::_s_jaccard_index_threshold = 0.5;
double FrameStatistics::_s_jaccard_index_percentile = 0.9;
double FrameStatistics::_s_cover_rate = 0.6;
bool FrameStatistics::_s_roi_is_main_lanes = false;

double cal_point_based_jaccard_index(unsigned int n1, unsigned int n2,
                                     unsigned int overlap) {
  if (n1 + n2 <= overlap) {
    return 0;
  }
  return static_cast<double>(overlap) / static_cast<double>(n1 + n2 - overlap);
}

void FrameStatistics::set_jaccard_index_threshold(double threshold) {
  _s_jaccard_index_threshold = threshold;
}

void FrameStatistics::set_jaccard_index_percentile(double percentile) {
  _s_jaccard_index_percentile = percentile;
}

void FrameStatistics::set_roi_is_main_lanes(bool value) {
  _s_roi_is_main_lanes = value;
}

double FrameStatistics::jaccard_index_percentile() const {
  if (_jaccard_indices.empty()) {
    return 0.0;
  }
  std::size_t id = static_cast<std::size_t>(
      static_cast<double>((_jaccard_indices.size() - 1)) *
      _s_jaccard_index_percentile);
  if (id < _jaccard_indices.size()) {
    return _jaccard_indices[id];
  } else {
    return 0.0;
  }
}

bool FrameStatistics::find_association() {
  std::size_t objects_num = objects.size();
  std::size_t gt_objects_num = gt_objects.size();

  std::vector<int> gt_object_id_per_point(_point_cloud->points.size(), -1);
  std::vector<bool> is_obj_matched(objects_num, false);
  std::vector<unsigned int> gt_object_match_num(gt_objects_num, 0);
  std::vector<std::map<unsigned int, unsigned int>> intersections(
      gt_objects_num);

  // fill gt_object_id_per_point
  for (std::size_t i = 0; i < gt_objects.size(); ++i) {
    for (auto& id : gt_objects[i]->indices->indices) {
      gt_object_id_per_point[id] = static_cast<int>(i);
    }
  }

  // calculate intersections
  for (unsigned int i = 0; i < objects.size(); ++i) {
    for (auto& id : objects[i]->indices->indices) {
      if (gt_object_id_per_point[id] >= 0) {
        ++intersections[gt_object_id_per_point[id]][i];
        ++gt_object_match_num[gt_object_id_per_point[id]];
      }
    }
  }

  _matches.clear();
  _strict_match_indices.clear();
  _isolated_object_indices_2017.clear();
  _isolated_gt_object_indices_2017.clear();
  _underseg_gt_object_indices_2017.clear();
  _isolated_gt_object_indices_2016.clear();
  _object_position.clear();
  _gt_object_position.clear();
  _jaccard_indices.clear();

  // find best matches and fill _strict_match_indices
  // fill _matches, _isolated_gt_object_indices_2017 and
  // _isolated_object_indices_2017
  ObjectMatch match;
  std::vector<std::vector<unsigned int>> grt_match_ids(gt_objects.size());
  std::vector<std::vector<unsigned int>> det_match_ids(objects.size());
  int match_id = 0;
  for (unsigned int i = 0; i < gt_objects.size(); ++i) {
    // 1. no match found with objects
    if (intersections[i].empty()) {
      _isolated_gt_object_indices_2017.push_back(i);
      continue;
    }
    match.first = i;  // match.first record gt object id
    // 2. record matches if exists overlapping points
    double best_jaccard_index = 0.0;
    ObjectMatch best_match;
    for (auto iter = intersections[i].begin(); iter != intersections[i].end();
         ++iter) {
      match.second = iter->first;  // match.second record object id
      match.matched_point_num = iter->second;
      match.jaccard_index = cal_point_based_jaccard_index(
          static_cast<unsigned int>(
              objects[match.second]->indices->indices.size()),
          static_cast<unsigned int>(
              gt_objects[match.first]->indices->indices.size()),
          match.matched_point_num);
      match.jaccard_index_over_gt =
          static_cast<double>(match.matched_point_num) /
          static_cast<double>(gt_objects[match.first]->indices->indices.size());
      match.confidence = static_cast<double>(objects[match.second]->confidence);
      if (match.jaccard_index > objects[match.second]->ji) {
        objects[match.second]->ji = match.jaccard_index;
      }
      if (match.jaccard_index > best_jaccard_index) {
        best_jaccard_index = match.jaccard_index;
        best_match = match;
      }
    }

    // record best match for each object
    _matches.push_back(best_match);
    grt_match_ids[best_match.first].push_back(match_id);
    det_match_ids[best_match.second].push_back(match_id);
    match_id++;
    // only accept strict match with jaccard index exceed threshold
    if (best_jaccard_index > _s_jaccard_index_threshold) {
      _strict_match_indices.push_back(
          static_cast<unsigned int>(_matches.size() - 1));
      is_obj_matched[_matches.back().second] = true;
    } else {
      _isolated_gt_object_indices_2017.push_back(i);
    }
  }
  // 3. record missed objects 2017
  for (unsigned int i = 0; i < objects.size(); ++i) {
    if (!is_obj_matched[i]) {
      _isolated_object_indices_2017.push_back(i);
    }
  }

  // 4. record missed groundtruth objects 2016
  for (unsigned int i = 0; i < gt_objects.size(); ++i) {
    if (static_cast<double>(gt_object_match_num[i]) /
            static_cast<double>(gt_objects[i]->indices->indices.size()) <=
        _s_cover_rate) {
      _isolated_gt_object_indices_2016.push_back(i);
    }
  }

  // 5. calculate object distance to origin (0, 0, 0)
  _object_position.resize(objects.size());
  _gt_object_position.resize(gt_objects.size());
  _orientation_similarity.resize(objects.size());
  _position_option.roi_is_main_lanes = _s_roi_is_main_lanes;
  for (std::size_t i = 0; i < gt_objects.size(); ++i) {
    _gt_object_position[i].cal_position_metric(gt_objects[i], _position_option);
  }
  for (auto& i : _strict_match_indices) {
    auto& m = _matches[i];
    _object_position[m.second] = _gt_object_position[m.first];
    _orientation_similarity[m.second].cal_orientation_similarity(
        objects[m.second], gt_objects[m.first]);
  }
  for (std::size_t i = 0; i < objects.size(); ++i) {
    if (!_object_position[i].is_valid) {
      _object_position[i].cal_position_metric(objects[i], _position_option);
    }
  }
  // 6. record all jaccard indices
  std::size_t valid_jaccard_indices_num = _matches.size();
  // note there may exist many to one matches
  std::size_t isolated_det_object_num =
      (valid_jaccard_indices_num >= objects_num)
          ? 0
          : objects_num - valid_jaccard_indices_num;
  std::size_t isolated_gt_object_num =
      gt_objects_num - valid_jaccard_indices_num;
  _jaccard_indices.resize(isolated_gt_object_num + isolated_det_object_num,
                          0.0);
  for (auto& match : _matches) {
    _jaccard_indices.push_back(match.jaccard_index);
  }
  std::sort(_jaccard_indices.begin(), _jaccard_indices.end());
  // 7. record under-segmented gt object indices
  for (auto& i : _isolated_gt_object_indices_2017) {
    if (grt_match_ids[i].empty()) {
      continue;
    }
    ObjectMatch& match = _matches[grt_match_ids[i][0]];
    if (match.jaccard_index > 0 && det_match_ids[match.second].size() > 1) {
      // && match.jaccard_index_over_gt > 0.5) {
      _underseg_gt_object_indices_2017.push_back(i);
    }
  }

  return true;
}
bool FrameStatistics::cal_meta_statistics() {
  _meta_stat.reset();
  std::vector<unsigned int> gt_object_range_indices(_gt_object_position.size(),
                                                    0);
  std::vector<unsigned int> object_range_indices(_object_position.size(), 0);
  // filling _total_detection_num and _total_groundtruth_num
  for (std::size_t i = 0; i < _object_position.size(); ++i) {
    auto& d = _object_position[i];
    object_range_indices[i] = MetaStatistics::get_range_index(d);
    ++_meta_stat._total_detection_num[object_range_indices[i]];
    // orientation
    _meta_stat._total_yaw_angle_diff[object_range_indices[i]] +=
        _orientation_similarity[i].delta;
  }
  for (std::size_t i = 0; i < _gt_object_position.size(); ++i) {
    auto& d = _gt_object_position[i];
    gt_object_range_indices[i] = MetaStatistics::get_range_index(d);
    ++_meta_stat._total_groundtruth_num[gt_object_range_indices[i]];
    if (gt_objects[i]->visible) {
      ++_meta_stat._total_visible_groundtruth_num[gt_object_range_indices[i]];
    }
  }
  std::vector<int> gt_object_indicator;
  // filling _total_ji_match_num
  gt_object_indicator.assign(_gt_object_position.size(), 1);
  for (auto& i : _isolated_gt_object_indices_2017) {
    gt_object_indicator[i] = -1;
  }
  for (std::size_t i = 0; i < gt_object_indicator.size(); ++i) {
    if (gt_object_indicator[i] > 0) {
      ++_meta_stat._total_ji_match_num[gt_object_range_indices[i]];
      if (gt_objects[i]->visible) {
        ++_meta_stat._total_visible_ji_match_num[gt_object_range_indices[i]];
      }
    }
  }
  // filling _total_hit_match_num
  gt_object_indicator.assign(_gt_object_position.size(), 1);
  for (auto& i : _isolated_gt_object_indices_2016) {
    gt_object_indicator[i] = -1;
  }
  for (std::size_t i = 0; i < gt_object_indicator.size(); ++i) {
    if (gt_object_indicator[i] > 0) {
      ++_meta_stat._total_hit_match_num[gt_object_range_indices[i]];
    }
  }
  // filling _total_ji_sum
  std::vector<double> gt_object_max_ji(_gt_object_position.size(), 0.0);
  for (auto& m : _matches) {
    unsigned int& id = m.first;
    gt_object_max_ji[id] = std::max(gt_object_max_ji[id], m.jaccard_index);
  }
  for (std::size_t i = 0; i < _gt_object_position.size(); ++i) {
    _meta_stat._total_ji_sum[gt_object_range_indices[i]] += gt_object_max_ji[i];
  }
  // filling _cumulated_match_num_per_conf (assume one-to-one match)
  for (auto& i : _strict_match_indices) {
    auto& m = _matches[i];
    if (object_range_indices[m.second] ==
        (MetaStatistics::get_range_dim() - 1)) {
      continue;
    }
    unsigned int conf_id = MetaStatistics::get_confidence_index(m.confidence);
    unsigned int gt_type_id =
        MetaStatistics::get_type_index(gt_objects[m.first]->type);
    ++_meta_stat._cumulated_match_num_per_conf[gt_type_id][conf_id];
    // OrientationSimilarityMetric osm;
    // osm.cal_orientation_similarity(objects[m.second], gt_objects[m.first]);
    // _meta_stat._cumulated_orientation_similarity_sum_per_conf[conf_id] +=
    // osm.similarity;
    _meta_stat._cumulated_orientation_similarity_sum_per_conf[conf_id] +=
        _orientation_similarity[m.second].similarity;
  }

  for (unsigned int t = 0; t < MetaStatistics::get_type_dim(); ++t) {
    for (int i = static_cast<int>(
             _meta_stat._cumulated_match_num_per_conf[t].size() - 2);
         i >= 0; --i) {
      _meta_stat._cumulated_match_num_per_conf[t][i] +=
          _meta_stat._cumulated_match_num_per_conf[t][i + 1];
    }
  }

  for (int i = static_cast<int>(
           _meta_stat._cumulated_orientation_similarity_sum_per_conf.size() -
           2);
       i >= 0; --i) {
    _meta_stat._cumulated_orientation_similarity_sum_per_conf[i] +=
        _meta_stat._cumulated_orientation_similarity_sum_per_conf[i + 1];
  }
  // filling _confusion_matrix
  for (auto& i : _strict_match_indices) {
    auto& m = _matches[i];
    unsigned int gt_type_id =
        MetaStatistics::get_type_index(gt_objects[m.first]->type);
    unsigned int obj_type_id =
        MetaStatistics::get_type_index(objects[m.second]->type);
    unsigned int& range_id = gt_object_range_indices[m.first];
    ++_meta_stat._confusion_matrix[range_id][gt_type_id][obj_type_id];
  }
  // filling _det_alone and _cumulated_det_alone_per_conf
  for (auto& i : _isolated_object_indices_2017) {
    unsigned int obj_type_id = MetaStatistics::get_type_index(objects[i]->type);
    unsigned int& range_id = object_range_indices[i];
    ++_meta_stat._det_alone[range_id][obj_type_id];
    if (object_range_indices[i] == (MetaStatistics::get_range_dim() - 1)) {
      continue;
    }
    unsigned int conf_id = MetaStatistics::get_confidence_index(
        static_cast<double>(objects[i]->confidence));
    ++_meta_stat._cumulated_det_alone_per_conf[obj_type_id][conf_id];
  }
  for (unsigned int t = 0; t < MetaStatistics::get_type_dim(); ++t) {
    for (int i = static_cast<int>(
             _meta_stat._cumulated_det_alone_per_conf[t].size() - 2);
         i >= 0; --i) {
      _meta_stat._cumulated_det_alone_per_conf[t][i] +=
          _meta_stat._cumulated_det_alone_per_conf[t][i + 1];
    }
  }
  // filling _gt_alone
  for (auto& i : _isolated_gt_object_indices_2017) {
    unsigned int gt_type_id =
        MetaStatistics::get_type_index(gt_objects[i]->type);
    unsigned int& range_id = gt_object_range_indices[i];
    ++_meta_stat._gt_alone[range_id][gt_type_id];
  }
  // filling _underseg_gt_num
  for (auto& i : _underseg_gt_object_indices_2017) {
    if (gt_object_range_indices[i] == (MetaStatistics::get_range_dim() - 1)) {
      continue;
    }
    unsigned int gt_type_id =
        MetaStatistics::get_type_index(gt_objects[i]->type);
    ++_meta_stat._underseg_gt_num[gt_type_id];
  }
  return true;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
