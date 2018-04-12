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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/probabilistic_fusion.h"

#include <iomanip>

#include "modules/common/macro.h"
#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_hm_track_object_matcher.h"

namespace apollo {
namespace perception {

using apollo::common::util::GetProtoFromFile;

ProbabilisticFusion::~ProbabilisticFusion() {
  if (matcher_) {
    delete matcher_;
    matcher_ = nullptr;
  }
}

bool ProbabilisticFusion::Init() {
  sensor_manager_ = PbfSensorManager::instance();
  ACHECK(sensor_manager_ != nullptr)
      << "Failed to get PbfSensorManager instance";
  track_manager_ = PbfTrackManager::instance();
  ACHECK(track_manager_ != nullptr) << "Failed to get PbfTrackManager instance";

  CHECK(GetProtoFromFile(FLAGS_probabilistic_fusion_config_file, &config_));

  // matching parameters
  std::string match_method = config_.match_method();
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

  float max_match_distance = config_.max_match_distance();
  ADEBUG << "Probabilistic_fusion max_match_distance: " << max_match_distance;
  PbfBaseTrackObjectMatcher::SetMaxMatchDistance(max_match_distance);

  // track related parameters
  float max_lidar_invisible_period = config_.max_lidar_invisible_period();
  PbfTrack::SetMaxLidarInvisiblePeriod(max_lidar_invisible_period);
  ADEBUG << "max_lidar_invisible_period: " << max_lidar_invisible_period;

  float max_radar_invisible_period = config_.max_radar_invisible_period();
  PbfTrack::SetMaxRadarInvisiblePeriod(max_radar_invisible_period);
  ADEBUG << "max_radar_invisible_period: " << max_radar_invisible_period;

  float max_radar_confident_angle = config_.max_radar_confident_angle();
  PbfTrack::SetMaxRadarConfidentAngle(max_radar_confident_angle);
  ADEBUG << "max_radar_confident_angle: " << max_radar_confident_angle;

  float min_radar_confident_distance = config_.min_radar_confident_distance();
  PbfTrack::SetMinRadarConfidentDistance(min_radar_confident_distance);
  AINFO << "min_radar_confident_distance: " << min_radar_confident_distance;

  bool publish_if_has_lidar = config_.publish_if_has_lidar();
  PbfTrack::SetPublishIfHasLidar(publish_if_has_lidar);
  ADEBUG << "publish_if_has_lidar: "
         << (publish_if_has_lidar ? "true" : "false");

  bool publish_if_has_radar = config_.publish_if_has_radar();
  PbfTrack::SetPublishIfHasRadar(publish_if_has_radar);
  ADEBUG << "publish_if_has_radar: "
         << (publish_if_has_radar ? "true" : "false");

  // publish driven
  publish_sensor_id_ = FLAGS_fusion_publish_sensor_id;
  if (publish_sensor_id_ != "velodyne_64" && publish_sensor_id_ != "radar" &&
      publish_sensor_id_ != "camera") {
    AERROR << "Invalid publish_sensor value: " << publish_sensor_id_;
  }
  ADEBUG << "publish_sensor: " << publish_sensor_id_;

  use_radar_ = config_.use_radar();
  ADEBUG << "use_radar :" << use_radar_;
  use_lidar_ = config_.use_lidar();
  ADEBUG << "use_lidar:" << use_lidar_;

  ADEBUG << "ProbabilisticFusion initialize successfully";
  return true;
}

bool ProbabilisticFusion::Fuse(
    const std::vector<SensorObjects> &multi_sensor_objects,
    std::vector<std::shared_ptr<Object>> *fused_objects) {
  ACHECK(fused_objects != nullptr) << "parameter fused_objects is nullptr";

  std::vector<PbfSensorFramePtr> frames;
  double fusion_time = 0;
  {
    sensor_data_rw_mutex_.lock();
    bool need_to_fusion = false;
    // 1. collect sensor objects data
    for (size_t i = 0; i < multi_sensor_objects.size(); ++i) {
      auto sensor_type = multi_sensor_objects[i].sensor_type;
      AINFO << "add sensor measurement: " << GetSensorType(sensor_type)
            << ", obj_cnt : " << multi_sensor_objects[i].objects.size() << ", "
            << std::fixed << std::setprecision(12)
            << multi_sensor_objects[i].timestamp;
      if (is_lidar(sensor_type) && !use_lidar_) {
        continue;
      }
      if (is_radar(sensor_type) && !use_radar_) {
        continue;
      }
      if (is_camera(sensor_type) && !use_camera_) {
        continue;
      }

      if (GetSensorType(multi_sensor_objects[i].sensor_type) ==
          publish_sensor_id_) {
        need_to_fusion = true;
        fusion_time = multi_sensor_objects[i].timestamp;
        started_ = true;
        sensor_manager_->AddSensorMeasurements(multi_sensor_objects[i]);
      } else if (started_) {
        sensor_manager_->AddSensorMeasurements(multi_sensor_objects[i]);
      }
    }

    if (!need_to_fusion) {
      sensor_data_rw_mutex_.unlock();
      return true;
    }

    // 2.query related sensor frames for fusion
    sensor_manager_->GetLatestFrames(fusion_time, &frames);
    sensor_data_rw_mutex_.unlock();
    AINFO << "Get " << frames.size() << " related frames for fusion";
  }

  {
    fusion_mutex_.lock();
    // 3.peform fusion on related frames
    for (size_t i = 0; i < frames.size(); ++i) {
      FuseFrame(frames[i]);
    }

    // 4.collect results
    CollectFusedObjects(fusion_time, fused_objects);
    fusion_mutex_.unlock();
  }

  return true;
}

std::string ProbabilisticFusion::name() const { return "ProbabilisticFusion"; }

void ProbabilisticFusion::FuseFrame(const PbfSensorFramePtr &frame) {
  AINFO << "Fusing frame: " << frame->sensor_id << ","
        << "object_number: " << frame->objects.size() << ","
        << "timestamp: " << std::fixed << std::setprecision(12)
        << frame->timestamp;
  std::vector<std::shared_ptr<PbfSensorObject>> &objects = frame->objects;
  std::vector<std::shared_ptr<PbfSensorObject>> background_objects;
  std::vector<std::shared_ptr<PbfSensorObject>> foreground_objects;
  DecomposeFrameObjects(objects, &foreground_objects, &background_objects);

  Eigen::Vector3d ref_point = frame->sensor2world_pose.topRightCorner(3, 1);
  FuseForegroundObjects(&foreground_objects, ref_point, frame->sensor_type,
                        frame->sensor_id, frame->timestamp);
  track_manager_->RemoveLostTracks();
}

void ProbabilisticFusion::CreateNewTracks(
    const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
    const std::vector<int> &unassigned_ids) {
  for (size_t i = 0; i < unassigned_ids.size(); i++) {
    int id = unassigned_ids[i];
    PbfTrackPtr track(new PbfTrack(sensor_objects[id]));
    track_manager_->AddTrack(track);
  }
}

void ProbabilisticFusion::UpdateAssignedTracks(
    std::vector<PbfTrackPtr> *tracks,
    const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
    const std::vector<std::pair<int, int>> &assignments,
    const std::vector<double> &track_object_dist) {
  for (size_t i = 0; i < assignments.size(); i++) {
    int local_track_index = assignments[i].first;
    int local_obj_index = assignments[i].second;
    (*tracks)[local_track_index]->UpdateWithSensorObject(
        sensor_objects[local_obj_index], track_object_dist[local_track_index]);
  }
}

void ProbabilisticFusion::UpdateUnassignedTracks(
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

void ProbabilisticFusion::CollectFusedObjects(
    double timestamp, std::vector<std::shared_ptr<Object>> *fused_objects) {
  if (fused_objects == nullptr) {
    return;
  }
  fused_objects->clear();

  int fg_obj_num = 0;
  std::vector<PbfTrackPtr> &tracks = track_manager_->GetTracks();
  for (size_t i = 0; i < tracks.size(); i++) {
    if (tracks[i]->AbleToPublish()) {
      std::shared_ptr<PbfSensorObject> fused_object =
          tracks[i]->GetFusedObject();
      std::shared_ptr<Object> obj(new Object());
      obj->clone(*(fused_object->object));
      obj->track_id = tracks[i]->GetTrackId();
      obj->latest_tracked_time = timestamp;
      obj->tracking_time = tracks[i]->GetTrackingPeriod();
      fused_objects->push_back(obj);
      fg_obj_num++;
    }
  }

  AINFO << "fg_track_cnt = " << tracks.size();
  AINFO << "collect objects : fg_obj_cnt = " << fg_obj_num
        << ", timestamp = " << GLOG_TIMESTAMP(timestamp);
}

void ProbabilisticFusion::DecomposeFrameObjects(
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

void ProbabilisticFusion::FuseForegroundObjects(
    std::vector<std::shared_ptr<PbfSensorObject>> *foreground_objects,
    Eigen::Vector3d ref_point, const SensorType &sensor_type,
    const std::string &sensor_id, double timestamp) {
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

  ADEBUG << "fg_track_cnt = " << tracks.size()
         << ", fg_obj_cnt = " << foreground_objects->size()
         << ", assignement = " << assignments.size()
         << ", unassigned_track_cnt = " << unassigned_tracks.size()
         << ", unassigned_obj_cnt = " << unassigned_objects.size();

  UpdateAssignedTracks(&tracks, *foreground_objects, assignments,
                       track2measurements_dist);

  UpdateUnassignedTracks(&tracks, unassigned_tracks, track2measurements_dist,
                         sensor_type, sensor_id, timestamp);

  if (FLAGS_use_navigation_mode) {
    if (is_camera(sensor_type)) {
      CreateNewTracks(*foreground_objects, unassigned_objects);
    }
  } else {
    CreateNewTracks(*foreground_objects, unassigned_objects);
  }
}

}  // namespace perception
}  // namespace apollo
