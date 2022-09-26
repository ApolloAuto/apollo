/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/fusion/lib/gatekeeper/collect_fused_object.h"

#include "modules/common/util/string_util.h"
#include "modules/perception/base/object_supplement.h"
#include "modules/perception/base/object.h"
#include "modules/perception/fusion/base/sensor_object.h"
#include "modules/perception/fusion/base/track.h"
#include "modules/perception/pipeline/plugin_factory.h"

namespace apollo {
namespace perception {
namespace fusion {


bool CollectFusedObject::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  // create plugins
  gate_keeper_ =
      pipeline::dynamic_unique_cast<BaseGatekeeper>(
          pipeline::PluginFactory::CreatePlugin(
              plugin_config_map_[PluginType::PBF_GATEKEEPER]));
  CHECK_NOTNULL(gate_keeper_);
  return true;
}

bool CollectFusedObject::Process(DataFrame* data_frame) {
  if (data_frame == nullptr)
    return false;

  FusionFrame* fusion_frame = data_frame->fusion_frame;
  if (fusion_frame == nullptr)
    return false;

  base::FrameConstPtr sensor_frame = fusion_frame->frame;
  if (sensor_frame == nullptr)
    return false;

  double fusion_time = sensor_frame->timestamp;
  scenes_ = fusion_frame->scene_ptr;
  Process(fusion_time, &fusion_frame->fused_objects);

  return true;
}

void CollectFusedObject::Process(
    double timestamp, std::vector<base::ObjectPtr>* fused_objects) {
  fused_objects->clear();

  size_t fg_obj_num = 0;
  const std::vector<TrackPtr>& foreground_tracks =
      scenes_->GetForegroundTracks();
  for (const auto& track_ptr : foreground_tracks) {
    if (gate_keeper_->AbleToPublish(track_ptr)) {
      CollectObjectsByTrack(timestamp, track_ptr, fused_objects);
      ++fg_obj_num;
    }
  }

  size_t bg_obj_num = 0;
  const std::vector<TrackPtr>& background_tracks =
      scenes_->GetBackgroundTracks();
  for (const auto& track_ptr : background_tracks) {
    if (gate_keeper_->AbleToPublish(track_ptr)) {
      CollectObjectsByTrack(timestamp, track_ptr, fused_objects);
      ++bg_obj_num;
    }
  }

  AINFO << "collect objects : fg_obj_cnt = " << fg_obj_num
        << ", bg_obj_cnt = " << bg_obj_num
        << ", timestamp = " << FORMAT_TIMESTAMP(timestamp);
}

void CollectFusedObject::CollectObjectsByTrack(
    double timestamp,
    const TrackPtr& track,
    std::vector<base::ObjectPtr>* fused_objects) {
  const FusedObjectPtr& fused_object = track->GetFusedObject();
  base::ObjectPtr obj = base::ObjectPool::Instance().Get();
  *obj = *(fused_object->GetBaseObject());

  // create obj->fusion_supplement.measurements
  const SensorId2ObjectMap& lidar_measurements = track->GetLidarObjects();
  const SensorId2ObjectMap& camera_measurements = track->GetCameraObjects();
  const SensorId2ObjectMap& radar_measurements = track->GetRadarObjects();

  size_t num_measurements = lidar_measurements.size() +
                            camera_measurements.size() +
                            radar_measurements.size();
  obj->fusion_supplement.on_use = true;
  std::vector<base::SensorObjectMeasurement>& measurements =
      obj->fusion_supplement.measurements;
  measurements.resize(num_measurements);

  // fill measurements
  size_t m_index = 0;
  for (const auto& iter : lidar_measurements) {
    CollectSensorMeasurementFromObject(iter.second, &measurements[m_index]);
    ++m_index;
  }
  for (const auto& iter : camera_measurements) {
    CollectSensorMeasurementFromObject(iter.second, &measurements[m_index]);
    ++m_index;
  }
  for (const auto& iter : radar_measurements) {
    CollectSensorMeasurementFromObject(iter.second, &measurements[m_index]);
    ++m_index;
  }

  // save to fused_objects
  obj->track_id = track->GetTrackId();
  obj->latest_tracked_time = timestamp;
  obj->tracking_time = track->GetTrackingPeriod();
  fused_objects->emplace_back(obj);

  ADEBUG << "fusion_reporting..." << obj->track_id << "@"
         << FORMAT_TIMESTAMP(timestamp) << "@(" << std::setprecision(10)
         << obj->center(0) << ","
         << obj->center(1) << ","
         << obj->center_uncertainty(0, 0) << ","
         << obj->center_uncertainty(0, 1) << ","
         << obj->center_uncertainty(1, 0) << ","
         << obj->center_uncertainty(1, 1) << ","
         << obj->velocity(0) << ","
         << obj->velocity(1) << ","
         << obj->velocity_uncertainty(0, 0) << ","
         << obj->velocity_uncertainty(0, 1) << ","
         << obj->velocity_uncertainty(1, 0) << ","
         << obj->velocity_uncertainty(1, 1) << ")";
}

void CollectFusedObject::CollectSensorMeasurementFromObject(
    const SensorObjectConstPtr& object,
    base::SensorObjectMeasurement* measurement) {
  measurement->sensor_id = object->GetSensorId();
  measurement->timestamp = object->GetTimestamp();
  measurement->track_id = object->GetBaseObject()->track_id;
  measurement->center = object->GetBaseObject()->center;
  measurement->theta = object->GetBaseObject()->theta;
  measurement->size = object->GetBaseObject()->size;
  measurement->velocity = object->GetBaseObject()->velocity;
  measurement->type = object->GetBaseObject()->type;
  if (IsCamera(object)) {
    measurement->box = object->GetBaseObject()->camera_supplement.box;
  }
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
