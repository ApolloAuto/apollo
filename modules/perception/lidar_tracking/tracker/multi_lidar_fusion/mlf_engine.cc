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

#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_engine.h"

#include <utility>

#include "Eigen/Geometry"

#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/common/algorithm/geometry/roi_filter.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/util.h"
#include "modules/perception/lidar_tracking/tracker/common/track_pool_types.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/util.h"

namespace apollo {
namespace perception {
namespace lidar {

void MlfEngine::Clear() {
  foreground_objects_.clear();
  background_objects_.clear();
  foreground_track_data_.clear();
  background_track_data_.clear();
}

bool MlfEngine::Init(const MultiTargetTrackerInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  MlfEngineConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  Clear();

  use_histogram_for_match_ = config.use_histogram_for_match();
  histogram_bin_size_ = config.histogram_bin_size();
  output_predict_objects_ = config.output_predict_objects();
  reserved_invisible_time_ = config.reserved_invisible_time();
  use_frame_timestamp_ = config.use_frame_timestamp();
  print_debug_log_ = config.print_debug_log();
  delay_output_ = config.delay_time_output();
  pub_track_times_ = config.pub_track_times();
  merge_foreground_background_ = config.merge_foreground_background();

  matcher_.reset(new MlfTrackObjectMatcher);
  MlfTrackObjectMatcherInitOptions matcher_init_options;
  matcher_init_options.config_path = options.config_path;
  ACHECK(matcher_->Init(matcher_init_options));

  tracker_.reset(new MlfTracker);
  MlfTrackerInitOptions tracker_init_options;
  tracker_init_options.config_path = options.config_path;
  ACHECK(tracker_->Init(tracker_init_options));
  return true;
}

bool MlfEngine::Track(const MultiTargetTrackerOptions& options,
                      LidarFrame* frame) {
  // 0. modify objects timestamp if necessary
  if (use_frame_timestamp_) {
    for (auto& object : frame->segmented_objects) {
      object->latest_tracked_time = frame->timestamp;
    }
  }
  if (print_debug_log_) {
    ObjectsDebugInfo(frame, true);
    ObjectsDebugInfo(frame, false);
  }
  // 1. add global offset to pose (only when no track exists)
  if (foreground_track_data_.empty() && background_track_data_.empty()) {
    global_to_local_offset_ = -frame->lidar2world_pose.translation();
  }
  sensor_to_local_pose_ = frame->lidar2world_pose;
  sensor_to_local_pose_.pretranslate(global_to_local_offset_);
  // 2. split fg and bg objects, and transform to tracked objects
  SplitAndTransformToTrackedObjects(frame->segmented_objects,
                                    frame->sensor_info, frame->timestamp);
  // 3. assign tracked objects to tracks
  MlfTrackObjectMatcherOptions match_options;
  TrackObjectMatchAndAssign(match_options, foreground_objects_, "foreground",
                            &foreground_track_data_);
  TrackObjectMatchAndAssign(match_options, background_objects_, "background",
                            &background_track_data_);
  // 4. state filter in tracker if is main sensor
  bool is_main_sensor = algorithm::SensorManager::Instance()->IsMainSensor(
      frame->sensor_info.name);
  if (is_main_sensor) {
    TrackStateFilter(foreground_track_data_, frame->timestamp);
    TrackStateFilter(background_track_data_, frame->timestamp);
  }
  // 5. track to object if is main sensor
  if (print_debug_log_) {
    TrackDebugInfo(frame);
  }
  frame->tracked_objects.clear();
  if (is_main_sensor) {
    CollectTrackedResult(frame);
  }
  // 6. remove stale data
  RemoveStaleTrackData("foreground", frame->timestamp, &foreground_track_data_);
  RemoveStaleTrackData("background", frame->timestamp, &background_track_data_);

  // Startegy: Set velocity and acceleration to ZERO outside hdmap_struct
  // temporarily located here, best is in mlf_motion_refiner.cc
  auto roi = frame->hdmap_struct;
  if (roi == nullptr ||
      (roi->road_polygons.empty() && roi->junction_polygons.empty() &&
       roi->road_boundary.empty())) {
    AINFO << "MlfEngine publish objects: " << frame->tracked_objects.size()
          << " sensor_name: " << frame->sensor_info.name
          << " at timestamp: " << frame->timestamp;
    return true;
  }
  std::stringstream sstr;
  sstr << "Set objects velocity to zero. track_id: ";
  for (auto obj : frame->tracked_objects) {
    if (obj->motion_state == base::MotionState::MOVING) {
      continue;
    }
    if (algorithm::IsObjectInRoi(roi, obj) &&
        obj->lidar_supplement.semantic_type != base::ObjectSemanticType::WALL &&
        obj->lidar_supplement.semantic_type != base::ObjectSemanticType::FENCE
        ) {
      continue;
    }
    obj->velocity = Eigen::Vector3f::Zero();
    obj->acceleration = Eigen::Vector3f::Zero();
    sstr << obj->track_id << ", ";
  }
  AINFO << sstr.str();

  AINFO << "MlfEngine publish objects: " << frame->tracked_objects.size()
        << " sensor_name: " << frame->sensor_info.name
        << " at timestamp: " << std::to_string(frame->timestamp);
  return true;
}

void MlfEngine::SplitAndTransformToTrackedObjects(
    const std::vector<base::ObjectPtr>& objects,
    const base::SensorInfo& sensor_info, double frame_timestamp) {
  std::vector<TrackedObjectPtr> tracked_objects;
  TrackedObjectPool::Instance().BatchGet(objects.size(), &tracked_objects);
  foreground_objects_.clear();
  background_objects_.clear();
  for (size_t i = 0; i < objects.size(); ++i) {
    double tracked_time = objects[i]->latest_tracked_time;
    double timestamp = use_frame_timestamp_ ? frame_timestamp : tracked_time;
    tracked_objects[i]->AttachObject(objects[i], sensor_to_local_pose_,
                      global_to_local_offset_, sensor_info, timestamp);

    if (merge_foreground_background_) {
      foreground_objects_.push_back(tracked_objects[i]);
    } else {
      if (!objects[i]->lidar_supplement.is_clustered &&
          use_histogram_for_match_) {
        tracked_objects[i]->histogram_bin_size = histogram_bin_size_;
        tracked_objects[i]->ComputeShapeFeatures();
      }

      if (!objects[i]->lidar_supplement.is_clustered) {
        foreground_objects_.push_back(tracked_objects[i]);
      } else {
        background_objects_.push_back(tracked_objects[i]);
      }
    }
  }
  AINFO << "MlfEngine: " << sensor_info.name
        << " foreground: " << foreground_objects_.size()
        << " background: " << background_objects_.size();
}

void MlfEngine::TrackObjectMatchAndAssign(
    const MlfTrackObjectMatcherOptions& match_options,
    const std::vector<TrackedObjectPtr>& objects, const std::string& name,
    std::vector<MlfTrackDataPtr>* tracks) {
  std::vector<std::pair<size_t, size_t>> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_objects;
  matcher_->Match(match_options, objects, *tracks, &assignments,
                  &unassigned_tracks, &unassigned_objects);
  AINFO << "MlfEngine: " + name + " assignments " << assignments.size()
        << " unassigned_tracks " << unassigned_tracks.size()
        << " unassigned_objects " << unassigned_objects.size();
  // 1. for assignment, push object to cache of track_data
  for (auto& pair : assignments) {
    const size_t track_id = pair.first;
    const size_t object_id = pair.second;
    tracks->at(track_id)->PushTrackedObjectToCache(objects[object_id]);
  }
  // 2. for unassigned_objects, create new tracks
  for (auto& id : unassigned_objects) {
    MlfTrackDataPtr track_data = MlfTrackDataPool::Instance().Get();
    tracker_->InitializeTrack(track_data, objects[id]);
    tracks->push_back(track_data);
  }
}

void MlfEngine::TrackStateFilter(const std::vector<MlfTrackDataPtr>& tracks,
                                 double frame_timestamp) {
  std::vector<TrackedObjectPtr> objects;
  for (auto& track_data : tracks) {
    track_data->GetAndCleanCachedObjectsInTimeInterval(&objects);
    for (auto& obj : objects) {
      tracker_->UpdateTrackDataWithObject(track_data, obj);
    }
    if (objects.empty()) {
      tracker_->UpdateTrackDataWithoutObject(frame_timestamp, track_data);
    }
  }
}

void MlfEngine::CollectTrackedResult(LidarFrame* frame) {
  auto& tracked_objects = frame->tracked_objects;
  tracked_objects.clear();
  size_t num_objects =
      foreground_track_data_.size() + background_track_data_.size();
  base::ObjectPool::Instance().BatchGet(num_objects, &tracked_objects);
  size_t pos = 0;
  size_t num_predict = 0;
  size_t num_delay_output = 0;
  size_t num_front_critical_reserve = 0;
  size_t num_blind_trafficcone = 0;
  auto collect = [&](std::vector<MlfTrackDataPtr>* tracks) {
    for (auto& track_data : *tracks) {
      // if (!output_predict_objects_ &&
      //      track_data->is_current_state_predicted_) {
      //   ++num_predict;
      // } else {
      //   if (!track_data->ToObject(-global_to_local_offset_,
      //        frame->timestamp, tracked_objects[pos], true)) {
      //     AERROR << "Tracking failed";
      //     continue;
      //   }
      //   ++pos;
      // }
      if (track_data->age_ <= pub_track_times_) {
          if (track_data->is_front_critical_track_) {
              ADEBUG << "[DelayOutput] track_id: " << track_data->track_id_
                    << " time is " << std::to_string(frame->timestamp)
                    << " not output";
          }
          ++num_delay_output;
          continue;
      }
      track_data->is_reserve_blind_cone_ = false;
      // == false -> OUTPUT
      if (!track_data->is_current_state_predicted_) {
          if (!track_data->ToObject(-global_to_local_offset_,
              frame->timestamp, tracked_objects[pos], true)) {
              AERROR << "Tracking failed";
              continue;
          }
          ++pos;
          ADEBUG << "track_id: " << track_data->track_id_
                 << " detetcted, obj-time is "
                 << std::to_string(frame->timestamp) << " and output";
          continue;
      } else {
          // == true: output_predict_objects_ == true -> OUTPUT
          if (output_predict_objects_) {
              if (!track_data->ToObject(-global_to_local_offset_,
                  frame->timestamp, tracked_objects[pos], true)) {
                  AERROR << "Tracking failed";
                  continue;
              }
              ++pos;
              continue;
          } else {
              // output_predict_objects_ == false
              // should judge front-critical and time
              TrackedObjectConstPtr latest_object =
                  track_data->GetLatestObject().second;
              // front-critical and within time -> OUTPUT
              if (latest_object != nullptr &&
                  track_data->is_front_critical_track_ &&
                  latest_object->output_velocity.head<2>().norm() < 0.01 &&
                  frame->timestamp -
                      track_data->GetLatestObject().first <= delay_output_) {
                  ++num_front_critical_reserve;
                  AINFO << "track_id: " << track_data->track_id_
                        << " missed, obj-time is "
                        << std::to_string(track_data->GetLatestObject().first)
                        << " and predict output";
                  if (!track_data->ToObject(-global_to_local_offset_,
                       frame->timestamp, tracked_objects[pos], false)) {
                      AERROR << "Tracking failed";
                      continue;
                  }
                  ++pos;
                  continue;
              } else if (JudgeBlindTrafficCone(track_data, frame->timestamp,
                  -global_to_local_offset_, frame->lidar2world_pose,
                  frame->lidar2novatel_extrinsics)) {
                  ++num_blind_trafficcone;
                  if (!track_data->ToObject(-global_to_local_offset_,
                       frame->timestamp, tracked_objects[pos], false)) {
                      AERROR << "Tracking failed";
                      continue;
                  }
                  track_data->is_reserve_blind_cone_ = true;
                  ++pos;
                  continue;
              } else {
                  // NO front-critical or beyond time -> NO OUTPUT
                  ++num_predict;
                  continue;
              }
          }
      }
    }
  };
  collect(&foreground_track_data_);
  collect(&background_track_data_);
  AINFO << "MlfEngine, num_predict: " << num_predict
        << " num delay_output: " << num_delay_output
        << " num front_critical: " << num_front_critical_reserve
        << " num blind trafficcone: " << num_blind_trafficcone
        << " num_objects: " << num_objects;
  if (num_predict > num_objects) {
    AERROR << "num_predict > num_objects";
    return;
  }
  tracked_objects.resize(num_objects - num_predict - num_delay_output);
}

void MlfEngine::RemoveStaleTrackData(const std::string& name, double timestamp,
                                     std::vector<MlfTrackDataPtr>* tracks) {
  size_t pos = 0;
  for (size_t i = 0; i < tracks->size(); ++i) {
    float reserve_time = reserved_invisible_time_;
    if (tracks->at(i)->is_front_critical_track_ &&
        reserved_invisible_time_ < delay_output_) {
        reserve_time = delay_output_;
    }
    if (tracks->at(i)->latest_visible_time_ + reserve_time >= timestamp ||
        tracks->at(i)->is_reserve_blind_cone_) {
      if (i != pos) {
        tracks->at(pos) = tracks->at(i);
      }
      ++pos;
    }
  }
  AINFO << "MlfEngine: " << name << " remove stale tracks, from "
        << tracks->size() << " to " << pos;
  tracks->resize(pos);
}

void MlfEngine::ObjectsDebugInfo(LidarFrame* frame, bool foreground_log) {
    size_t objcnt = 0;
    for (auto obj : frame->segmented_objects) {
        if (foreground_log && !obj->lidar_supplement.is_background) {
            objcnt++;
        }
        if (!foreground_log && obj->lidar_supplement.is_background) {
            objcnt++;
        }
    }
    std::stringstream ssstr;
    if (foreground_log) {
        ssstr << "[Foreground-objects] timestamp: "
              << std::to_string(frame->timestamp) << " objs: " << objcnt
              << std::endl;
    } else {
        ssstr << "[Background-objects] timestamp: "
              << std::to_string(frame->timestamp) << " objs: " << objcnt
              << std::endl;
    }
    for (auto obj : frame->segmented_objects) {
        if (foreground_log && obj->lidar_supplement.is_background) {
            continue;
        }
        if (!foreground_log && !obj->lidar_supplement.is_background) {
            continue;
        }
        ssstr << "id = " << obj->id << ": " << obj->center(0) << ", "
              << obj->center(1) << ", " << obj->center(2) << ", "
              << obj->size(0) << ", " << obj->size(1) << ", "
              << obj->size(2) << ", " << obj->theta << ", "
              << static_cast<int>(obj->type) << std::endl;
    }
    AINFO << ssstr.str();
}

void MlfEngine::TrackDebugInfo(LidarFrame* frame) {
    std::stringstream sstr;
    sstr << "[This frame TrackDataDebugInfo]: timestamp: "
         << std::to_string(frame->timestamp) << " tracks: "
         << foreground_track_data_.size() << std::endl;
    sstr << "lidar2world_pose: " << sensor_to_local_pose_(0, 0) << ", "
         << sensor_to_local_pose_(0, 1) << ", " << sensor_to_local_pose_(0, 2)
         << ", " << sensor_to_local_pose_(0, 3) << ", "
         << sensor_to_local_pose_(1, 0) << ", " << sensor_to_local_pose_(1, 1)
         << ", " << sensor_to_local_pose_(1, 2) << ", "
         << sensor_to_local_pose_(1, 3) << ", " << sensor_to_local_pose_(2, 0)
         << ", " << sensor_to_local_pose_(2, 1) << ", "
         << sensor_to_local_pose_(2, 2) << ", " << sensor_to_local_pose_(2, 3)
         << ", "<< sensor_to_local_pose_(3, 0) << ", "
         << sensor_to_local_pose_(3, 1) << ", " << sensor_to_local_pose_(3, 2)
         << ", " << sensor_to_local_pose_(3, 3) << ", "
         << std::to_string(global_to_local_offset_(0)) << ", "
         << std::to_string(global_to_local_offset_(1)) << ", "
         << std::to_string(global_to_local_offset_(2)) << std::endl;

    auto debug_info = [&](std::vector<MlfTrackDataPtr>* tracks) {
        for (auto& track_data : *tracks) {
            const TrackedObjectConstPtr latest_obj =
                track_data->GetLatestObject().second;
            sstr << " track_id = " << track_data->track_id_ << ": "
                 << latest_obj->center(0) << ", " << latest_obj->center(1)
                 << ", " << latest_obj->center(2) << ", "
                 << latest_obj->size(0) << ", " << latest_obj->size(1) << ", "
                 << latest_obj->size(2) << ", "
                 << latest_obj->output_direction(0) << ", "
                 << latest_obj->output_direction(1) << ", "
                 << latest_obj->output_direction(2) << ", "
                 << static_cast<int>(latest_obj->type) << ", "
                 << std::to_string(latest_obj->object_ptr->latest_tracked_time)
                 << std::endl;
        }
    };
    debug_info(&foreground_track_data_);
    AINFO << sstr.str();
}

PERCEPTION_REGISTER_MULTITARGET_TRACKER(MlfEngine);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
