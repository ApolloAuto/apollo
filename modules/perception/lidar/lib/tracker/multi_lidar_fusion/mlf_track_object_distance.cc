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

#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_track_object_distance.h"

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/lib/tracker/association/distance_collection.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

// location dist weight, irection dist weight, bbox size dist weight,
// point num dist weight, histogram dist weight, centroid shift dist weight
// bbox iou dist weight
const std::vector<float> MlfTrackObjectDistance::kForegroundDefaultWeight = {
    0.6f, 0.2f, 0.1f, 0.1f, 0.5f, 0.f, 0.f};
// location dist weight, irection dist weight, bbox size dist weight,
// point num dist weight, histogram dist weight, centroid shift dist weight
// bbox iou dist weight
const std::vector<float> MlfTrackObjectDistance::kBackgroundDefaultWeight = {
    0.f, 0.f, 0.f, 0.f, 0.f, 0.2f, 0.8f};

bool MlfTrackObjectDistance::Init(
    const MlfTrackObjectDistanceInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = cyber::common::GetAbsolutePath(work_root, root_path);
  config_file = cyber::common::GetAbsolutePath(
      config_file, "mlf_track_object_distance.conf");
  MlfDistanceConfig config;
  CHECK(cyber::common::GetProtoFromFile(config_file, &config));

  foreground_weight_table_.clear();
  background_weight_table_.clear();
  for (int i = 0; i < config.foreground_weights_size(); ++i) {
    const auto& fgws = config.foreground_weights(i);
    const std::string& name = fgws.sensor_name_pair();
    std::vector<float> weights(7, 0.f);
    weights[0] = fgws.location_dist_weight();
    weights[1] = fgws.direction_dist_weight();
    weights[2] = fgws.bbox_size_dist_weight();
    weights[3] = fgws.point_num_dist_weight();
    weights[4] = fgws.histogram_dist_weight();
    weights[5] = fgws.centroid_shift_dist_weight();
    weights[6] = fgws.bbox_iou_dist_weight();
    foreground_weight_table_.emplace(name, weights);
  }
  for (int i = 0; i < config.background_weights_size(); ++i) {
    const auto& bgws = config.background_weights(i);
    const std::string& name = bgws.sensor_name_pair();
    std::vector<float> weights(7, 0.f);
    weights[0] = bgws.location_dist_weight();
    weights[1] = bgws.direction_dist_weight();
    weights[2] = bgws.bbox_size_dist_weight();
    weights[3] = bgws.point_num_dist_weight();
    weights[4] = bgws.histogram_dist_weight();
    weights[5] = bgws.centroid_shift_dist_weight();
    weights[6] = bgws.bbox_iou_dist_weight();
    background_weight_table_.emplace(name, weights);
  }

  return true;
}

float MlfTrackObjectDistance::ComputeDistance(
    const TrackedObjectConstPtr& object,
    const MlfTrackDataConstPtr& track) const {
  bool is_background = object->is_background;
  const TrackedObjectConstPtr latest_object = track->GetLatestObject().second;
  std::string key = latest_object->sensor_info.name + object->sensor_info.name;
  const std::vector<float>* weights = nullptr;
  if (is_background) {
    auto iter = background_weight_table_.find(key);
    if (iter == background_weight_table_.end()) {
      weights = &kBackgroundDefaultWeight;
    } else {
      weights = &iter->second;
    }
  } else {
    auto iter = foreground_weight_table_.find(key);
    if (iter == foreground_weight_table_.end()) {
      weights = &kForegroundDefaultWeight;
    } else {
      weights = &iter->second;
    }
  }
  if (weights == nullptr || weights->size() < 7) {
    AERROR << "Invalid weights";
    return 1e+10f;
  }
  float distance = 0.f;
  float delta = 1e-10f;

  double current_time = object->object_ptr->latest_tracked_time;
  track->PredictState(current_time);

  double time_diff =
      track->age_ ? current_time - track->latest_visible_time_ : 0;
  if (weights->at(0) > delta) {
    distance +=
        weights->at(0) * LocationDistance(latest_object, track->predict_.state,
                                          object, time_diff);
  }
  if (weights->at(1) > delta) {
    distance +=
        weights->at(1) * DirectionDistance(latest_object, track->predict_.state,
                                           object, time_diff);
  }
  if (weights->at(2) > delta) {
    distance +=
        weights->at(2) * BboxSizeDistance(latest_object, track->predict_.state,
                                          object, time_diff);
  }
  if (weights->at(3) > delta) {
    distance +=
        weights->at(3) * PointNumDistance(latest_object, track->predict_.state,
                                          object, time_diff);
  }
  if (weights->at(4) > delta) {
    distance +=
        weights->at(4) * HistogramDistance(latest_object, track->predict_.state,
                                           object, time_diff);
  }
  if (weights->at(5) > delta) {
    distance += weights->at(5) * CentroidShiftDistance(latest_object,
                                                       track->predict_.state,
                                                       object, time_diff);
  }
  if (weights->at(6) > delta) {
    distance += weights->at(6) *
                BboxIouDistance(latest_object, track->predict_.state, object,
                                time_diff, background_object_match_threshold_);
  }
  return distance;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
