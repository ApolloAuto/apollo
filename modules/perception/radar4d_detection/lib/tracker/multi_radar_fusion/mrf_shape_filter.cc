/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/mrf_shape_filter.h"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/radar/common/radar_object_util.h"
#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/proto/mrf_config.pb.h"

namespace apollo {
namespace perception {
namespace radar4d {

bool MrfShapeFilter::Init(const MrfFilterInitOptions& options) {
  std::string config_file = "mrf_shape_filter.pb.txt";
  if (!options.config_file.empty()) {
    config_file = options.config_file;
  }
  config_file = GetConfigFile(options.config_path, config_file);
  MrfShapeFilterConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  bottom_points_ignore_threshold_ = config.bottom_points_ignore_threshold();
  top_points_ignore_threshold_ = config.top_points_ignore_threshold();
  return true;
}

void MrfShapeFilter::UpdateWithObject(const MrfFilterOptions& options,
                                      const MrfTrackDataConstPtr& track_data,
                                      TrackedObjectPtr new_object) {
  // compute tight object polygon
  auto& obj = new_object->object_ptr;
  hull_.GetConvexHull(obj->radar4d_supplement.cloud_world, &obj->polygon);

  // simple moving average orientation filtering
  if (track_data->age_ > 0) {
    TrackedObjectConstPtr latest_object = track_data->GetLatestObject().second;
    if (new_object->direction.dot(latest_object->direction) < 0) {
      new_object->direction *= -1;
    }
    static const double kMovingAverage = 0.6;
    new_object->direction =
        latest_object->output_direction * (1 - kMovingAverage) +
        new_object->direction * kMovingAverage;
    new_object->direction.normalize();
  }
  Eigen::Vector3f local_direction = obj->direction;
  Eigen::Vector3d local_center = obj->center;
  Eigen::Vector3f local_size = obj->size;
  obj->direction = new_object->direction.cast<float>();  // sync
  // finally, recompute object shape
  ComputeObjectShapeFromPolygon(obj, true);
  new_object->center = obj->center;
  new_object->size = obj->size.cast<double>();
  // center and size in object should not changed
  obj->center = local_center;
  obj->size = local_size;
  obj->direction = local_direction;
  new_object->output_center = new_object->center;
  new_object->output_direction = new_object->direction;
  new_object->output_size = new_object->size;
}

void MrfShapeFilter::UpdateWithoutObject(const MrfFilterOptions& options,
                                         double timestamp,
                                         MrfTrackDataPtr track_data) {
  // TODO(.)
}

PERCEPTION_REGISTER_MRFFILTER(MrfShapeFilter);

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
