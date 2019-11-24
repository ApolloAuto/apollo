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

#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_shape_filter.h"

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_object_util.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;

bool MlfShapeFilter::Init(const MlfFilterInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, "mlf_shape_filter.conf");
  MlfShapeFilterConfig config;
  CHECK(cyber::common::GetProtoFromFile(config_file, &config));

  bottom_points_ignore_threshold_ = config.bottom_points_ignore_threshold();
  top_points_ignore_threshold_ = config.top_points_ignore_threshold();
  return true;
}

void MlfShapeFilter::UpdateWithObject(const MlfFilterOptions& options,
                                      const MlfTrackDataConstPtr& track_data,
                                      TrackedObjectPtr new_object) {
  // compute tight object polygon
  auto& obj = new_object->object_ptr;
  if (new_object->is_background) {
    hull_.GetConvexHull(obj->lidar_supplement.cloud_world, &obj->polygon);
  } else {
    hull_.GetConvexHullWithoutGroundAndHead(
        obj->lidar_supplement.cloud_world,
        static_cast<float>(bottom_points_ignore_threshold_),
        static_cast<float>(top_points_ignore_threshold_), &obj->polygon);
  }
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

void MlfShapeFilter::UpdateWithoutObject(const MlfFilterOptions& options,
                                         double timestamp,
                                         MlfTrackDataPtr track_data) {
  // TODO(.)
}

PERCEPTION_REGISTER_MLFFILTER(MlfShapeFilter);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
