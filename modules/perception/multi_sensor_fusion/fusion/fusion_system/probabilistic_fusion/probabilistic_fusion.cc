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
#include "modules/perception/multi_sensor_fusion/fusion/fusion_system/probabilistic_fusion/probabilistic_fusion.h"

#include <map>
#include <utility>

#include "modules/perception/multi_sensor_fusion/proto/probabilistic_fusion_config.pb.h"

#include "cyber/common/file.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/util.h"
#include "modules/perception/multi_sensor_fusion/base/track_pool_types.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association/hm_tracks_objects_match.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/tracker/pbf_tracker/pbf_tracker.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h"
#include "modules/perception/multi_sensor_fusion/fusion/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h"

namespace apollo {
namespace perception {
namespace fusion {

using cyber::common::GetAbsolutePath;

bool ProbabilisticFusion::Init(const FusionInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  ProbabilisticFusionConfig params;
  if (!cyber::common::GetProtoFromFile(config_file, &params)) {
    AERROR << "Read config failed: " << config_file;
    return false;
  }

  params_.use_lidar = params.use_lidar();
  params_.use_radar = params.use_radar();
  params_.use_camera = params.use_camera();
  for (int i = 0; i < params.prohibition_sensors_size(); ++i) {
    params_.prohibition_sensors.push_back(params.prohibition_sensors(i));
  }

  // static member initialization from PB config
  Track::SetMaxLidarInvisiblePeriod(params.max_lidar_invisible_period());
  Track::SetMaxRadarInvisiblePeriod(params.max_radar_invisible_period());
  Track::SetMaxCameraInvisiblePeriod(params.max_camera_invisible_period());
  Sensor::SetMaxCachedFrameNumber(params.max_cached_frame_num());

  scenes_.reset(new Scene());

  // Init data association
  auto data_association_param = params.data_association_param();
  AssociationInitOptions association_init_options;
  association_init_options.config_path = data_association_param.config_path();
  association_init_options.config_file = data_association_param.config_file();
  BaseDataAssociation* matcher =
      BaseDataAssociationRegisterer::GetInstanceByName(
          data_association_param.name());
  CHECK_NOTNULL(matcher);
  matcher_.reset(matcher);
  ACHECK(matcher_->Init(association_init_options))
      << "Fusion matcher init error";

  // Init gatekeeper
  auto gatekeeper_param = params.gatekeeper_param();
  GatekeeperInitOptions gatekeeper_init_options;
  gatekeeper_init_options.config_path = gatekeeper_param.config_path();
  gatekeeper_init_options.config_file = gatekeeper_param.config_file();
  BaseGatekeeper* gate_keeper =
      BaseGatekeeperRegisterer::GetInstanceByName(gatekeeper_param.name());
  CHECK_NOTNULL(gate_keeper);
  gate_keeper_.reset(gate_keeper);
  ACHECK(gate_keeper_->Init(gatekeeper_init_options))
      << "Fusion gate_keeper init error";

  // Init pbftracker
  auto tracker_param = params.tracker_param();
  TrackerInitOptions tracker_init_options;
  // TODO(zero): pbftracker is dynamically created, so only set the name.
  params_.tracker_method = tracker_param.name();
  tracker_init_options.config_path = tracker_param.config_path();
  tracker_init_options.config_file = tracker_param.config_file();
  bool state = PbfTracker::InitParams(tracker_init_options);

  return state;
}

bool ProbabilisticFusion::Fuse(const base::FrameConstPtr& sensor_frame,
                               std::vector<base::ObjectPtr>* fused_objects) {
  if (fused_objects == nullptr) {
    AERROR << "fusion error: fused_objects is nullptr";
    return false;
  }

  auto* sensor_data_manager = SensorDataManager::Instance();
  // 1. save frame data
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    if (!params_.use_lidar && sensor_data_manager->IsLidar(sensor_frame)) {
      return true;
    }
    if (!params_.use_radar && sensor_data_manager->IsRadar(sensor_frame)) {
      return true;
    }
    if (!params_.use_camera && sensor_data_manager->IsCamera(sensor_frame)) {
      return true;
    }

    bool is_publish_sensor = IsPublishSensor(sensor_frame);

    AINFO << "add sensor measurement: " << sensor_frame->sensor_info.name
          << ", obj_cnt : " << sensor_frame->objects.size() << ", "
          << FORMAT_TIMESTAMP(sensor_frame->timestamp);
    sensor_data_manager->AddSensorMeasurements(sensor_frame);

    if (!is_publish_sensor) {
      return true;
    }
  }

  // 2. query related sensor_frames for fusion
  std::lock_guard<std::mutex> fuse_lock(fuse_mutex_);
  double fusion_time = sensor_frame->timestamp;
  std::vector<SensorFramePtr> frames;
  sensor_data_manager->GetLatestFrames(fusion_time, &frames);
  AINFO << "Get " << frames.size() << " related frames for fusion";

  // 3. perform fusion on related frames
  for (const auto& frame : frames) {
    FuseFrame(frame);
  }

  // 4. collect fused objects
  CollectFusedObjects(fusion_time, fused_objects);
  return true;
}

bool ProbabilisticFusion::IsPublishSensor(
    const base::FrameConstPtr& sensor_frame) const {
  std::string sensor_id = sensor_frame->sensor_info.name;
  bool is_main_sensor =
      algorithm::SensorManager::Instance()->IsMainSensor(sensor_id);
  return is_main_sensor;
}

void ProbabilisticFusion::FuseFrame(const SensorFramePtr& frame) {
  AINFO << "Fusing frame: " << frame->GetSensorId()
        << ", foreground_object_number: "
        << frame->GetForegroundObjects().size()
        << ", background_object_number: "
        << frame->GetBackgroundObjects().size()
        << ", timestamp: " << FORMAT_TIMESTAMP(frame->GetTimestamp());
  FuseForegroundTrack(frame);
  FusebackgroundTrack(frame);
  RemoveLostTrack();
}

void ProbabilisticFusion::FuseForegroundTrack(const SensorFramePtr& frame) {
  AssociationOptions options;
  AssociationResult association_result;
  matcher_->Associate(options, frame, scenes_, &association_result);

  const std::vector<TrackMeasurmentPair>& assignments =
      association_result.assignments;
  UpdateAssignedTracks(frame, assignments);

  const std::vector<size_t>& unassigned_track_inds =
      association_result.unassigned_tracks;
  UpdateUnassignedTracks(frame, unassigned_track_inds);

  const std::vector<size_t>& unassigned_obj_inds =
      association_result.unassigned_measurements;
  CreateNewTracks(frame, unassigned_obj_inds);
}

void ProbabilisticFusion::UpdateAssignedTracks(
    const SensorFramePtr& frame,
    const std::vector<TrackMeasurmentPair>& assignments) {
  // Attention: match_distance should be used
  // in ExistenceFusion to calculate existence score.
  // We set match_distance to zero if track and object are matched,
  // which only has a small difference compared with actural match_distance
  TrackerOptions options;
  options.match_distance = 0;
  std::vector<SensorObjectPtr>& f_ground_objs = frame->GetForegroundObjects();
  for (size_t i = 0; i < assignments.size(); ++i) {
    size_t track_ind = assignments[i].first;
    size_t obj_ind = assignments[i].second;
    trackers_[track_ind]->UpdateWithMeasurement(options, f_ground_objs[obj_ind],
                                                frame->GetTimestamp());
  }
}

void ProbabilisticFusion::UpdateUnassignedTracks(
    const SensorFramePtr& frame,
    const std::vector<size_t>& unassigned_track_inds) {
  // Attention: match_distance(min_match_distance) should be used
  // in ExistenceFusion to calculate toic score.
  // Due to it hasn't been used(mainly for front radar object pub in
  // gatekeeper),
  // we do not set match_distance temporarily.
  TrackerOptions options;
  options.match_distance = 0;
  std::string sensor_id = frame->GetSensorId();
  for (size_t i = 0; i < unassigned_track_inds.size(); ++i) {
    size_t track_ind = unassigned_track_inds[i];
    trackers_[track_ind]->UpdateWithoutMeasurement(
        options, sensor_id, frame->GetTimestamp(), frame->GetTimestamp());
  }
}

void ProbabilisticFusion::CreateNewTracks(
    const SensorFramePtr& frame,
    const std::vector<size_t>& unassigned_obj_inds) {
  // check valid
  bool prohibition_sensor_flag = false;
  std::for_each(params_.prohibition_sensors.begin(),
                params_.prohibition_sensors.end(),
                [&](const std::string& sensor_name) {
                  if (sensor_name == frame->GetSensorId())
                    prohibition_sensor_flag = true;
                });
  if (prohibition_sensor_flag) {
    return;
  }

  // add new track
  std::vector<SensorObjectPtr>& f_ground_objs = frame->GetForegroundObjects();
  for (const size_t& obj_ind : unassigned_obj_inds) {
    TrackPtr track = TrackPool::Instance().Get();
    track->Initialize(f_ground_objs[obj_ind]);
    scenes_->AddForegroundTrack(track);

    ADEBUG << "object id: " << f_ground_objs[obj_ind]->GetBaseObject()->track_id
           << ", create new track: " << track->GetTrackId();

    if (params_.tracker_method == "PbfTracker") {
      std::shared_ptr<BaseTracker> tracker;
      tracker.reset(new PbfTracker());
      tracker->Init(track, f_ground_objs[obj_ind]);
      trackers_.emplace_back(tracker);
    }
  }
}

void ProbabilisticFusion::FusebackgroundTrack(const SensorFramePtr& frame) {
  // 1. association
  size_t track_size = scenes_->GetBackgroundTracks().size();
  size_t obj_size = frame->GetBackgroundObjects().size();
  std::map<int, size_t> local_id_2_track_ind_map;
  std::vector<bool> track_tag(track_size, false);
  std::vector<bool> object_tag(obj_size, false);

  std::vector<TrackPtr>& background_tracks = scenes_->GetBackgroundTracks();
  for (size_t i = 0; i < track_size; ++i) {
    const FusedObjectPtr& obj = background_tracks[i]->GetFusedObject();
    int local_id = obj->GetBaseObject()->track_id;
    local_id_2_track_ind_map[local_id] = i;
  }

  std::vector<TrackMeasurmentPair> assignments;
  std::vector<SensorObjectPtr>& frame_objs = frame->GetBackgroundObjects();
  for (size_t i = 0; i < obj_size; ++i) {
    int local_id = frame_objs[i]->GetBaseObject()->track_id;
    const auto& it = local_id_2_track_ind_map.find(local_id);
    if (it != local_id_2_track_ind_map.end()) {
      size_t track_ind = it->second;
      assignments.push_back(std::make_pair(track_ind, i));
      track_tag[track_ind] = true;
      object_tag[i] = true;
    }
  }

  // 2. update assigned track
  for (size_t i = 0; i < assignments.size(); ++i) {
    size_t track_ind = assignments[i].first;
    size_t obj_ind = assignments[i].second;
    background_tracks[track_ind]->UpdateWithSensorObject(frame_objs[obj_ind]);
  }

  // 3. update unassigned track
  std::string sensor_id = frame->GetSensorId();
  for (size_t i = 0; i < track_tag.size(); ++i) {
    if (!track_tag[i]) {
      background_tracks[i]->UpdateWithoutSensorObject(sensor_id,
                                                      frame->GetTimestamp());
    }
  }

  // 4. create new track
  for (size_t i = 0; i < object_tag.size(); ++i) {
    if (!object_tag[i]) {
      TrackPtr track = TrackPool::Instance().Get();
      track->Initialize(frame_objs[i], true);
      scenes_->AddBackgroundTrack(track);
    }
  }
}

void ProbabilisticFusion::RemoveLostTrack() {
  // need to remove tracker at the same time
  size_t f_alive_index = 0;
  std::vector<TrackPtr>& foreground_tracks = scenes_->GetForegroundTracks();
  for (size_t i = 0; i < foreground_tracks.size(); ++i) {
    if (foreground_tracks[i]->IsAlive()) {
      foreground_tracks[f_alive_index] = foreground_tracks[i];
      trackers_[f_alive_index] = trackers_[i];
      ++f_alive_index;
    }
  }
  AINFO << "Remove " << foreground_tracks.size() - f_alive_index
        << " foreground tracks. " << f_alive_index << " tracks left.";
  foreground_tracks.resize(f_alive_index);
  trackers_.resize(f_alive_index);

  // only need to remove frame track
  size_t b_alive_index = 0;
  std::vector<TrackPtr>& background_tracks = scenes_->GetBackgroundTracks();
  for (size_t i = 0; i < background_tracks.size(); ++i) {
    if (background_tracks[i]->IsAlive()) {
      background_tracks[b_alive_index] = background_tracks[i];
      ++b_alive_index;
    }
  }
  AINFO << "Remove " << background_tracks.size() - b_alive_index
        << " background tracks";
  background_tracks.resize(b_alive_index);
}

void ProbabilisticFusion::CollectFusedObjects(
    double timestamp, std::vector<base::ObjectPtr>* fused_objects) {
  CHECK_NOTNULL(fused_objects);
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

void ProbabilisticFusion::CollectObjectsByTrack(
    double timestamp, const TrackPtr& track,
    std::vector<base::ObjectPtr>* fused_objects) {
  CHECK_NOTNULL(track);
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
  auto& measurements = obj->fusion_supplement.measurements;
  measurements.resize(num_measurements);

  // fill measurements
  size_t m_index = 0;
  for (const auto& iter : lidar_measurements) {
    CollectSensorMeasurementFromObject(iter.second, &measurements[m_index++]);
  }
  for (const auto& iter : camera_measurements) {
    CollectSensorMeasurementFromObject(iter.second, &measurements[m_index++]);
  }
  for (const auto& iter : radar_measurements) {
    CollectSensorMeasurementFromObject(iter.second, &measurements[m_index++]);
  }

  // save to fused_objects
  obj->track_id = track->GetTrackId();
  obj->latest_tracked_time = timestamp;
  obj->tracking_time = track->GetTrackingPeriod();
  fused_objects->emplace_back(obj);

  ADEBUG << "fusion_reporting..." << obj->track_id << "@"
         << FORMAT_TIMESTAMP(timestamp) << "@(" << std::setprecision(10)
         << obj->center(0) << "," << obj->center(1) << ","
         << obj->center_uncertainty(0, 0) << ","
         << obj->center_uncertainty(0, 1) << ","
         << obj->center_uncertainty(1, 0) << ","
         << obj->center_uncertainty(1, 1) << "," << obj->velocity(0) << ","
         << obj->velocity(1) << "," << obj->velocity_uncertainty(0, 0) << ","
         << obj->velocity_uncertainty(0, 1) << ","
         << obj->velocity_uncertainty(1, 0) << ","
         << obj->velocity_uncertainty(1, 1) << ")";
}

void ProbabilisticFusion::CollectSensorMeasurementFromObject(
    const SensorObjectConstPtr& object,
    base::SensorObjectMeasurement* measurement) {
  CHECK_NOTNULL(object);
  CHECK_NOTNULL(measurement);
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

FUSION_REGISTER_FUSIONSYSTEM(ProbabilisticFusion);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
