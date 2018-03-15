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

#include "modules/perception/obstacle/fusion/async_fusion/async_fusion.h"

#include <iomanip>
#include <string>
#include <vector>
#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_hm_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"

namespace apollo {
namespace perception {

AsyncFusion::AsyncFusion()
    : started_(false), matcher_(nullptr), track_manager_(nullptr) {}

AsyncFusion::~AsyncFusion() {
  if (matcher_) {
    delete matcher_;
    matcher_ = nullptr;
  }
}

bool AsyncFusion::Init() {
  track_manager_ = PbfTrackManager::instance();
  ACHECK(track_manager_ != nullptr) << "Failed to get PbfTrackManager instance";
  const ModelConfig *model_config = NULL;
  if (!ConfigManager::instance()->GetModelConfig(name(), &model_config)) {
    AERROR << "not found model config: " << name();
    return false;
  }
  /**matching parameters*/
  std::string match_method = "hm_matcher";
  if (!model_config->GetValue("match_method", &match_method)) {
    AERROR << "match_method not found";
  }
  if (match_method == "hm_matcher") {
    matcher_ = new PbfHmTrackObjectMatcher();
    if (matcher_->Init()) {
      AINFO << "Initialize " << matcher_->name() << " successfully!";
    } else {
      AERROR << "Failed to initialize " << matcher_->name();
      return false;
    }
  } else {
    AERROR << "undefined match_method " << match_method
           << " and use default hm_matcher";
    matcher_ = new PbfHmTrackObjectMatcher();
    if (matcher_->Init()) {
      AINFO << "Initialize " << matcher_->name() << " successfully!";
    } else {
      AERROR << "Failed to initialize " << matcher_->name();
      return false;
    }
  }

  float max_match_distance = 4.0;
  if (!model_config->GetValue("max_match_distance", &max_match_distance)) {
    AERROR << "max_match_distance not found";
  }
  AINFO << "async_fusion max_match_distance: " << max_match_distance;
  PbfBaseTrackObjectMatcher::SetMaxMatchDistance(max_match_distance);
  return true;
}

PbfSensorFramePtr AsyncFusion::ConstructFrame(const SensorObjects &frame) {
  PbfSensorFramePtr pbf_frame(new PbfSensorFrame());
  pbf_frame->timestamp = frame.timestamp;
  pbf_frame->sensor2world_pose = frame.sensor2world_pose;
  pbf_frame->sensor_type = frame.sensor_type;
  pbf_frame->sensor_id = GetSensorType(frame.sensor_type);
  pbf_frame->seq_num = frame.seq_num;

  pbf_frame->objects.resize(frame.objects.size());
  for (size_t i = 0; i < frame.objects.size(); i++) {
    PbfSensorObjectPtr obj(new PbfSensorObject());
    obj->timestamp = frame.timestamp;
    obj->sensor_type = frame.sensor_type;
    obj->object->clone(*(frame.objects[i]));
    obj->sensor_id = GetSensorType(frame.sensor_type);
    pbf_frame->objects[i] = obj;
  }
  return pbf_frame;
}

bool AsyncFusion::Fuse(const std::vector<SensorObjects> &multi_sensor_objects,
                       std::vector<ObjectPtr> *fused_objects) {
  ACHECK(fused_objects != nullptr) << "parameter fused_objects is nullptr";
  ACHECK(multi_sensor_objects.size() == 1);

  // async fusion only process one fusion objects per time
  const SensorObjects &obj = multi_sensor_objects[0];

  double fusion_time = obj.timestamp;
  AINFO << "get sensor data " << GetSensorType(obj.sensor_type)
        << ", obj_cnt : " << obj.objects.size() << ", " << std::fixed
        << std::setprecision(12) << obj.timestamp;

  PbfSensorFramePtr frame = ConstructFrame(obj);

  {
    fusion_mutex_.lock();
    FuseFrame(frame);
    // 4.collect results, we don't need to collect fused_objects unless we have
    // to
    CollectFusedObjects(fusion_time, fused_objects);
    fusion_mutex_.unlock();
  }

  return true;
}

std::string AsyncFusion::name() const { return "AsyncFusion"; }

void AsyncFusion::FuseFrame(const PbfSensorFramePtr &frame) {
  AINFO << "Fusing frame: " << frame->sensor_id << ","
        << "object_number: " << frame->objects.size() << ","
        << "timestamp: " << std::fixed << std::setprecision(12)
        << frame->timestamp;
  std::vector<PbfSensorObjectPtr> &objects = frame->objects;
  std::vector<PbfSensorObjectPtr> background_objects;
  std::vector<PbfSensorObjectPtr> foreground_objects;
  DecomposeFrameObjects(objects, &foreground_objects, &background_objects);

  Eigen::Vector3d ref_point = frame->sensor2world_pose.topRightCorner(3, 1);
  FuseForegroundObjects(&foreground_objects, ref_point, frame->sensor_type,
                        frame->sensor_id, frame->timestamp);
  track_manager_->RemoveLostTracks();
}

void AsyncFusion::CreateNewTracks(
    const std::vector<PbfSensorObjectPtr> &sensor_objects,
    const std::vector<int> &unassigned_ids) {
  for (size_t i = 0; i < unassigned_ids.size(); i++) {
    int id = unassigned_ids[i];
    PbfTrackPtr track(new PbfTrack(sensor_objects[id]));
    track_manager_->AddTrack(track);
  }
}

void AsyncFusion::UpdateAssignedTracks(
    std::vector<PbfTrackPtr> *tracks,
    const std::vector<PbfSensorObjectPtr> &sensor_objects,
    const std::vector<TrackObjectPair> &assignments,
    const std::vector<double> &track_object_dist) {
  for (size_t i = 0; i < assignments.size(); i++) {
    int local_track_index = assignments[i].first;
    int local_obj_index = assignments[i].second;
    (*tracks)[local_track_index]->UpdateWithSensorObject(
        sensor_objects[local_obj_index], track_object_dist[local_track_index]);
  }
}

void AsyncFusion::UpdateUnassignedTracks(
    std::vector<PbfTrackPtr> *tracks, const std::vector<int> &unassigned_tracks,
    const std::vector<double> &track_object_dist, const SensorType &sensor_type,
    const std::string &sensor_id, double timestamp) {
  for (size_t i = 0; i < unassigned_tracks.size(); i++) {
    int local_track_index = unassigned_tracks[i];
    (*tracks)[local_track_index]->UpdateWithoutSensorObject(
        sensor_type, sensor_id, track_object_dist[local_track_index],
        timestamp);
  }
}

void AsyncFusion::CollectFusedObjects(double timestamp,
                                      std::vector<ObjectPtr> *fused_objects) {
  if (fused_objects == nullptr) {
    return;
  }
  fused_objects->clear();

  int fg_obj_num = 0;
  std::vector<PbfTrackPtr> &tracks = track_manager_->GetTracks();
  for (size_t i = 0; i < tracks.size(); i++) {
    if (tracks[i]->AbleToPublish()) {
      PbfSensorObjectPtr fused_object = tracks[i]->GetFusedObject();
      ObjectPtr obj(new Object());
      obj->clone(*(fused_object->object));
      obj->track_id = tracks[i]->GetTrackId();
      obj->latest_tracked_time = timestamp;
      obj->tracking_time = tracks[i]->GetTrackingPeriod();
      fused_objects->push_back(obj);
      fg_obj_num++;
    }
  }

  AINFO << "fg_track_cnt = " << tracks.size();
  AINFO << "collect objects : fg_obj_cnt = "
        << ", timestamp = " << GLOG_TIMESTAMP(timestamp);
}

void AsyncFusion::DecomposeFrameObjects(
    const std::vector<PbfSensorObjectPtr> &frame_objects,
    std::vector<PbfSensorObjectPtr> *foreground_objects,
    std::vector<PbfSensorObjectPtr> *background_objects) {
  foreground_objects->clear();
  background_objects->clear();
  for (size_t i = 0; i < frame_objects.size(); i++) {
    if (frame_objects[i]->object->is_background) {
      background_objects->push_back(frame_objects[i]);
    } else {
      foreground_objects->push_back(frame_objects[i]);
    }
  }
}

void AsyncFusion::FuseForegroundObjects(
    std::vector<PbfSensorObjectPtr> *foreground_objects,
    Eigen::Vector3d ref_point, const SensorType &sensor_type,
    const std::string &sensor_id, double timestamp) {
  std::vector<int> unassigned_tracks;
  std::vector<int> unassigned_objects;
  std::vector<TrackObjectPair> assignments;

  std::vector<PbfTrackPtr> &tracks = track_manager_->GetTracks();

  TrackObjectMatcherOptions options;
  options.ref_point = &ref_point;

  std::vector<double> track2measurements_dist;
  std::vector<double> measurement2tracks_dist;
  matcher_->Match(tracks, *foreground_objects, options, &assignments,
                  &unassigned_tracks, &unassigned_objects,
                  &track2measurements_dist, &measurement2tracks_dist);

  AINFO << "fg_track_cnt = " << tracks.size()
        << ", fg_obj_cnt = " << foreground_objects->size()
        << ", assignement = " << assignments.size()
        << ", unassigned_track_cnt = " << unassigned_tracks.size()
        << ", unassigned_obj_cnt = " << unassigned_objects.size();

  UpdateAssignedTracks(&tracks, *foreground_objects, assignments,
                       track2measurements_dist);

  UpdateUnassignedTracks(&tracks, unassigned_tracks, track2measurements_dist,
                         sensor_type, sensor_id, timestamp);

  // fixme:zhangweide only create new track if it is camera sensor
  if (is_camera(sensor_type)) {
      CreateNewTracks(*foreground_objects, unassigned_objects);
  }
}

}  // namespace perception
}  // namespace apollo
