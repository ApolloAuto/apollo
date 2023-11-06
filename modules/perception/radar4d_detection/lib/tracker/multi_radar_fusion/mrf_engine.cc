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

#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/mrf_engine.h"

#include <utility>
#include "Eigen/Geometry"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/prediction_msgs/feature.pb.h"
#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/proto/mrf_config.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/radar4d_detection/lib/tracker/common/track_pool_types.h"

namespace apollo {
namespace perception {
namespace radar4d {

using apollo::prediction::Feature;

void MrfEngine::Clear() {
  main_sensors_.clear();
  foreground_objects_.clear();
  background_objects_.clear();
  foreground_track_data_.clear();
  background_track_data_.clear();
}

bool MrfEngine::Init(const MultiTargetTrackerInitOptions& options) {
  std::string config_file = GetConfigFile(
      options.config_path, options.config_file);
  MrfEngineConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  Clear();

  for (int i = 0; i < config.main_sensor_size(); ++i) {
    main_sensors_.emplace(config.main_sensor(i));
  }

  // default value
  if (main_sensors_.empty()) {
    main_sensors_.emplace("radar_front");
  }

  use_histogram_for_match_ = config.use_histogram_for_match();
  histogram_bin_size_ = config.histogram_bin_size();
  output_predict_objects_ = config.output_predict_objects();
  reserved_invisible_time_ = config.reserved_invisible_time();
  use_frame_timestamp_ = config.use_frame_timestamp();

  matcher_.reset(new MrfTrackObjectMatcher);
  MrfTrackObjectMatcherInitOptions matcher_init_options;
  matcher_init_options.config_path = options.config_path;
  ACHECK(matcher_->Init(matcher_init_options));

  tracker_.reset(new MrfTracker);
  MrfTrackerInitOptions tracker_init_options;
  tracker_init_options.config_path = options.config_path;
  ACHECK(tracker_->Init(tracker_init_options));
  return true;
}

bool MrfEngine::Track(const MultiTargetTrackerOptions& options,
                      RadarFrame* frame) {
  // 0. modify objects timestamp if necessary
  if (use_frame_timestamp_) {
    for (auto& object : frame->segmented_objects) {
      object->latest_tracked_time = frame->timestamp;
    }
  }
  // 1. add global offset to pose (only when no track exists)
  if (foreground_track_data_.empty() && background_track_data_.empty()) {
    global_to_local_offset_ = -frame->radar2world_pose.translation();
  }
  sensor_to_local_pose_ = frame->radar2world_pose;
  sensor_to_local_pose_.pretranslate(global_to_local_offset_);
  // 2. split fg and bg objects, and transform to tracked objects
  SplitAndTransformToTrackedObjects(frame->segmented_objects,
                                    frame->sensor_info);
  // 3. assign tracked objects to tracks
  MrfTrackObjectMatcherOptions match_options;
  TrackObjectMatchAndAssign(match_options, foreground_objects_, "foreground",
                            &foreground_track_data_);
  TrackObjectMatchAndAssign(match_options, background_objects_, "background",
                            &background_track_data_);
  // 4. state filter in tracker if is main sensor
  bool is_main_sensor =
      (main_sensors_.find(frame->sensor_info.name) != main_sensors_.end());
  if (is_main_sensor) {
    TrackStateFilter(foreground_track_data_, frame->timestamp);
    TrackStateFilter(background_track_data_, frame->timestamp);
  }
  // 5. track to object if is main sensor
  frame->tracked_objects.clear();
  if (is_main_sensor) {
    CollectTrackedResult(frame);
  }
  // 6. remove stale data
  RemoveStaleTrackData("foreground", frame->timestamp, &foreground_track_data_);
  RemoveStaleTrackData("background", frame->timestamp, &background_track_data_);

  AINFO << "MrfEngine publish objects: " << frame->tracked_objects.size()
        << " sensor_name: " << frame->sensor_info.name
        << " at timestamp: " << frame->timestamp;
  return true;
}

void MrfEngine::SplitAndTransformToTrackedObjects(
    const std::vector<base::ObjectPtr>& objects,
    const base::SensorInfo& sensor_info) {
  std::vector<TrackedObjectPtr> tracked_objects;
  TrackedObjectPool::Instance().BatchGet(objects.size(), &tracked_objects);
  foreground_objects_.clear();
  background_objects_.clear();
  for (size_t i = 0; i < objects.size(); ++i) {
    tracked_objects[i]->AttachObject(objects[i], sensor_to_local_pose_,
                                     global_to_local_offset_, sensor_info);
    if (!objects[i]->radar4d_supplement.is_background &&
        use_histogram_for_match_) {
      tracked_objects[i]->histogram_bin_size = histogram_bin_size_;
      tracked_objects[i]->ComputeShapeFeatures();
    }
    if (objects[i]->radar4d_supplement.is_background) {
      background_objects_.push_back(tracked_objects[i]);
    } else {
      foreground_objects_.push_back(tracked_objects[i]);
    }
  }
  AINFO << "MrfEngine: " << sensor_info.name
        << " foreground: " << foreground_objects_.size()
        << " background: " << background_objects_.size();
}

void MrfEngine::TrackObjectMatchAndAssign(
    const MrfTrackObjectMatcherOptions& match_options,
    const std::vector<TrackedObjectPtr>& objects, const std::string& name,
    std::vector<MrfTrackDataPtr>* tracks) {
  std::vector<std::pair<size_t, size_t>> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_objects;
  matcher_->Match(match_options, objects, *tracks, &assignments,
                  &unassigned_tracks, &unassigned_objects);
  AINFO << "MrfEngine: " + name + " assignments " << assignments.size()
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
    MrfTrackDataPtr track_data = MrfTrackDataPool::Instance().Get();
    tracker_->InitializeTrack(track_data, objects[id]);
    tracks->push_back(track_data);
  }
}

void MrfEngine::TrackStateFilter(const std::vector<MrfTrackDataPtr>& tracks,
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

void convertPoseToLoc(const Eigen::Affine3d& pose,
                      localization::LocalizationEstimate* localization) {
  ADEBUG << "translation x y z " << pose.translation()[0] << " "
         << pose.translation()[1] << " " << pose.translation()[2];
  localization->mutable_pose()->mutable_position()->set_x(
      pose.translation()[0]);
  localization->mutable_pose()->mutable_position()->set_y(
      pose.translation()[1]);
  localization->mutable_pose()->mutable_position()->set_z(
      pose.translation()[2]);
  Eigen::Quaterniond p(pose.rotation());
  localization->mutable_pose()->mutable_orientation()->set_qx(p.x());
  localization->mutable_pose()->mutable_orientation()->set_qy(p.y());
  localization->mutable_pose()->mutable_orientation()->set_qz(p.z());
  localization->mutable_pose()->mutable_orientation()->set_qw(p.w());
}

// TODO(all): semantic map related, for debugging
// void MrfEngine::AttachDebugInfo(
//    std::vector<std::shared_ptr<base::Object>>* foreground_objs) {
//  for (auto i : obstacle_container_.curr_frame_movable_obstacle_ids()) {
//    Obstacle* obj = obstacle_container_.GetObstacle(i);
//    for (size_t i = 0; i < (*foreground_objs).size(); ++i) {
//      if (obj->id() == (*foreground_objs)[static_cast<int>(i)]->track_id) {
//        (*foreground_objs)[static_cast<int>(i)]->feature.reset(
//            new Feature(obj->latest_feature()));
//        ADEBUG << "traj size is mrf engine is "
//               << (*foreground_objs)[static_cast<int>(i)]
//                      ->feature->predicted_trajectory_size()
//               << " track id "
//               << (*foreground_objs)[static_cast<int>(i)]->track_id
//               << " feature address is "
//               << static_cast<void*>(
//                      (*foreground_objs)[static_cast<int>(i)]->feature.get());
//      }
//    }
//  }
//}

// TODO(all): semantic map related, for debugging
// void MrfEngine::AttachSemanticPredictedTrajectory(
//    const std::vector<MrfTrackDataPtr>& tracks) {
//  for (auto i : obstacle_container_.curr_frame_movable_obstacle_ids()) {
//    Obstacle* obj = obstacle_container_.GetObstacle(i);
//    for (size_t j = 0; j < tracks.size(); ++j) {
//      MrfTrackDataPtr ptr = tracks[j];
//      if (obj->id() == ptr->track_id_) {
//        ptr->feature_.reset(new Feature(obj->latest_feature()));
//      }
//    }
//  }
//}

void MrfEngine::CollectTrackedResult(RadarFrame* frame) {
  auto& tracked_objects = frame->tracked_objects;
  tracked_objects.clear();
  size_t num_objects =
      foreground_track_data_.size() + background_track_data_.size();
  base::ObjectPool::Instance().BatchGet(num_objects, &tracked_objects);
  size_t pos = 0;
  size_t num_predict = 0;
  auto collect = [&](std::vector<MrfTrackDataPtr>* tracks) {
    for (auto& track_data : *tracks) {
      if (!output_predict_objects_ && track_data->is_current_state_predicted_) {
        ++num_predict;
      } else {
        if (!track_data->ToObject(-global_to_local_offset_, frame->timestamp,
                                  tracked_objects[pos])) {
          AERROR << "Tracking failed";
          continue;
        }
        ++pos;
      }
    }
  };
  collect(&foreground_track_data_);
  // update semantic map object container
// TODO(all): semantic map related, for debugging
//  if (use_semantic_map_) {
//    obstacle_container_.CleanUp();
//    // use msg serializer to convert object to perception obstacles
//    apollo::common::ErrorCode err = apollo::common::ErrorCode::OK;
//    apollo::perception::PerceptionObstacles obstacles;
//    double radar_ts = frame->timestamp;
//    localization::LocalizationEstimate localization;
//    localization.mutable_header()->set_timestamp_sec(radar_ts);
//    localization.mutable_header()->set_radar_timestamp(radar_ts * 1e9);
//    localization.mutable_pose()->mutable_linear_velocity()->set_x(0.0f);
//    localization.mutable_pose()->mutable_linear_velocity()->set_y(0.0f);
//    localization.mutable_pose()->mutable_linear_velocity()->set_z(0.0f);
//    convertPoseToLoc(frame->novatel2world_pose, &localization);
//    pose_container_.Insert(localization);
//    obstacle_container_.InsertPerceptionObstacle(
//        *(pose_container_.ToPerceptionObstacle()), radar_ts);
//    std::vector<std::shared_ptr<base::Object>> foreground_objs(
//        tracked_objects.begin(), tracked_objects.begin() + pos);
//    serializer_.SerializeMsg(0, static_cast<uint64_t>(radar_ts * 1e9), 0,
//                             foreground_objs, err, &obstacles);
//    obstacle_container_.Insert(obstacles);
//    evaluator_.Run(&obstacle_container_);
//    AttachDebugInfo(&foreground_objs);
//    AttachSemanticPredictedTrajectory(foreground_track_data_);
//  }

  collect(&background_track_data_);
  if (num_predict != 0) {
    AINFO << "MrfEngine, num_predict: " << num_predict
          << " num_objects: " << num_objects;
    if (num_predict > num_objects) {
      AERROR << "num_predict > num_objects";
      return;
    }
    tracked_objects.resize(num_objects - num_predict);
  }
}

void MrfEngine::RemoveStaleTrackData(const std::string& name, double timestamp,
                                     std::vector<MrfTrackDataPtr>* tracks) {
  size_t pos = 0;
  for (size_t i = 0; i < tracks->size(); ++i) {
    if (tracks->at(i)->latest_visible_time_ + reserved_invisible_time_ >=
        timestamp) {
      if (i != pos) {
        tracks->at(pos) = tracks->at(i);
      }
      ++pos;
    }
  }
  AINFO << "MrfEngine: " << name << " remove stale tracks, from "
        << tracks->size() << " to " << pos;
  tracks->resize(pos);
}

PERCEPTION_REGISTER_MULTITARGET_TRACKER(MrfEngine);

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
