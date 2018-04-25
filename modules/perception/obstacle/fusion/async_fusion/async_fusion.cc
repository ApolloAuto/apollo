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

#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_hm_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"

namespace apollo {
namespace perception {

bool AsyncFusion::Init() {
  track_manager_ = PbfTrackManager::instance();
  CHECK_NOTNULL(track_manager_);
  const ModelConfig *model_config =
      ConfigManager::instance()->GetModelConfig(name());
  if (model_config == nullptr) {
    AERROR << "not found model config: " << name();
    return false;
  }
  /* matching parameters */
  // TODO(All): match_method is set to hm_matcher, so that line 56 - 65 is
  // redundant. We should either make match_method configurable or remove those
  // redundant code.
  std::string match_method = "hm_matcher";
  if (!model_config->GetValue("match_method", &match_method)) {
    AERROR << "match_method not found";
  }
  if (match_method == "hm_matcher") {
    matcher_.reset(new PbfHmTrackObjectMatcher());
    if (matcher_->Init()) {
      AINFO << "Initialize " << matcher_->name() << " successfully!";
    } else {
      AERROR << "Failed to initialize " << matcher_->name();
      return false;
    }
  } else {
    AERROR << "undefined match_method " << match_method
           << " and use default hm_matcher";
    matcher_.reset(new PbfHmTrackObjectMatcher());
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
  PbfTrack::SetMotionFusionMethod("PbfIMFFusion");
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
    std::shared_ptr<PbfSensorObject> obj(new PbfSensorObject());
    obj->timestamp = frame.timestamp;
    obj->sensor_type = frame.sensor_type;
    obj->object->clone(*(frame.objects[i]));
    obj->sensor_id = GetSensorType(frame.sensor_type);
    pbf_frame->objects[i] = obj;
  }
  return pbf_frame;
}

bool AsyncFusion::Fuse(const std::vector<SensorObjects> &multi_sensor_objects,
                       std::vector<std::shared_ptr<Object>> *fused_objects) {
  ACHECK(fused_objects != nullptr) << "parameter fused_objects is nullptr";

  AINFO << "number of sensor objects in async fusion is "
        << multi_sensor_objects.size();

  // process all the frames from one of the sensors
  for (const auto &obj : multi_sensor_objects) {
    double fusion_time = obj.timestamp;
    AINFO << "get sensor data " << GetSensorType(obj.sensor_type)
          << ", obj_cnt : " << obj.objects.size() << ", " << std::fixed
          << std::setprecision(12) << obj.timestamp;

    PbfSensorFramePtr frame = ConstructFrame(obj);

    {
      fusion_mutex_.lock();
      FuseFrame(frame);
      // 4.collect results
      CollectFusedObjects(fusion_time, fused_objects);
      fusion_mutex_.unlock();
    }
  }
  return true;
}

std::string AsyncFusion::name() const { return "AsyncFusion"; }

void AsyncFusion::FuseFrame(const PbfSensorFramePtr &frame) {
  AINFO << "Fusing frame: " << frame->sensor_id << ","
        << "object_number: " << frame->objects.size() << ","
        << "timestamp: " << std::fixed << std::setprecision(12)
        << frame->timestamp;
  std::vector<std::shared_ptr<PbfSensorObject>> &objects = frame->objects;
  std::vector<std::shared_ptr<PbfSensorObject>> background_objects;
  std::vector<std::shared_ptr<PbfSensorObject>> foreground_objects;
  DecomposeFrameObjects(objects, &foreground_objects, &background_objects);
  AINFO << "There are " << foreground_objects.size() << " foreground objects "
        << "\n " << background_objects.size() << " background objects";

  Eigen::Vector3d ref_point = frame->sensor2world_pose.topRightCorner(3, 1);
  FuseForegroundObjects(ref_point, frame->sensor_type, frame->sensor_id,
                        frame->timestamp, &foreground_objects);
  track_manager_->RemoveLostTracks();
}

void AsyncFusion::CreateNewTracks(
    const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
    const std::vector<int> &unassigned_ids) {
  for (size_t i = 0; i < unassigned_ids.size(); ++i) {
    int id = unassigned_ids[i];
    PbfTrackPtr track(new PbfTrack(sensor_objects[id]));
    track_manager_->AddTrack(track);
  }
}

void AsyncFusion::UpdateAssignedTracks(
    const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
    const std::vector<std::pair<int, int>> &assignments,
    const std::vector<double> &track_object_dist,
    std::vector<PbfTrackPtr> const *tracks) {
  for (size_t i = 0; i < assignments.size(); ++i) {
    int local_track_index = assignments[i].first;
    int local_obj_index = assignments[i].second;
    tracks->at(local_track_index)
        ->UpdateWithSensorObject(sensor_objects[local_obj_index],
                                 track_object_dist[local_track_index]);
  }
}

void AsyncFusion::UpdateUnassignedTracks(
    const std::vector<int> &unassigned_tracks,
    const std::vector<double> &track_object_dist, const SensorType &sensor_type,
    const std::string &sensor_id, const double timestamp,
    std::vector<PbfTrackPtr> *tracks) {
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    int local_track_index = unassigned_tracks[i];
    tracks->at(local_track_index)
        ->UpdateWithoutSensorObject(sensor_type, sensor_id,
                                    track_object_dist[local_track_index],
                                    timestamp);
  }
}

void AsyncFusion::CollectFusedObjects(
    double timestamp, std::vector<std::shared_ptr<Object>> *fused_objects) {
  if (fused_objects == nullptr) {
    return;
  }
  fused_objects->clear();

  int fg_obj_num = 0;
  std::vector<PbfTrackPtr> &tracks = track_manager_->GetTracks();
  for (size_t i = 0; i < tracks.size(); i++) {
    std::shared_ptr<PbfSensorObject> fused_object = tracks[i]->GetFusedObject();
    std::shared_ptr<Object> obj(new Object());
    obj->clone(*(fused_object->object));
    obj->track_id = tracks[i]->GetTrackId();
    obj->latest_tracked_time = timestamp;
    obj->tracking_time = tracks[i]->GetTrackingPeriod();
    fused_objects->push_back(obj);
    fg_obj_num++;
  }

  AINFO << "collect objects : fg_track_cnt = " << tracks.size()
        << ", timestamp = " << GLOG_TIMESTAMP(timestamp);
}

void AsyncFusion::DecomposeFrameObjects(
    const std::vector<std::shared_ptr<PbfSensorObject>> &frame_objects,
    std::vector<std::shared_ptr<PbfSensorObject>> *foreground_objects,
    std::vector<std::shared_ptr<PbfSensorObject>> *background_objects) {
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
    const Eigen::Vector3d &ref_point, const SensorType &sensor_type,
    const std::string &sensor_id, const double timestamp,
    std::vector<std::shared_ptr<PbfSensorObject>> *foreground_objects) {
  std::vector<int> unassigned_tracks;
  std::vector<int> unassigned_objects;
  std::vector<std::pair<int, int>> assignments;

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

  UpdateAssignedTracks(*foreground_objects, assignments,
                       track2measurements_dist, &tracks);

  UpdateUnassignedTracks(unassigned_tracks, track2measurements_dist,
                         sensor_type, sensor_id, timestamp, &tracks);

  // fixme:zhangweide only create new track if it is camera sensor
  if (is_camera(sensor_type)) {
    CreateNewTracks(*foreground_objects, unassigned_objects);
  }
}

}  // namespace perception
}  // namespace apollo
