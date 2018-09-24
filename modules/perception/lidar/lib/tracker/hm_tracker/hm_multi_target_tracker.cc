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
#include <iomanip>
#include <map>
#include <utility>

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/io/file_util.h"
#include "modules/perception/lib/io/protobuf_util.h"
#include "modules/perception/lidar/common/feature_descriptor.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/lidar_object_util.h"
#include "modules/perception/lidar/lib/tracker/common/track_pool_types.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/hm_multi_target_tracker.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/proto/hm_tracker_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

int HmMultiTargetTracker::s_maximum_consecutive_invisible_count_ = 1;
float HmMultiTargetTracker::s_minimum_visible_ratio_ = 0.6;

bool HmMultiTargetTracker::Init(const MultiTargetTrackerInitOptions& options) {
  lib::ConfigManager* config_manager =
      lib::Singleton<lib::ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager);
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = lib::FileUtil::GetAbsolutePath(work_root, root_path);
  config_file = lib::FileUtil::GetAbsolutePath(config_file,
                                               "hm_multi_target_tracker.conf");
  HmMultiTargetTrackerConfig config;
  CHECK(lib::ParseProtobufFromFile(config_file, &config));

  foreground_matcher_method_ = config.foreground_mathcer_method();
  background_matcher_method_ = config.background_matcher_method();
  use_histogram_for_match_ = config.use_histogram_for_match();

  // initialize tracker
  tracker_.reset(new Tracker);
  CHECK_NOTNULL(tracker_.get());

  // Initialize params of Tracker
  TrackerOption tracker_options;
  CHECK(tracker_->Init(tracker_options));

  foreground_matcher_.reset(new ObjectTrackMatcher);
  background_matcher_.reset(new ObjectTrackMatcher);
  CHECK_NOTNULL(foreground_matcher_.get());
  CHECK_NOTNULL(background_matcher_.get());

  ObjectTrackMatcherInitOptions foreground_matcher_init_options;
  foreground_matcher_init_options.is_background = false;
  foreground_matcher_init_options.matcher_name = foreground_matcher_method_;
  ObjectTrackMatcherInitOptions background_matcher_init_options;
  background_matcher_init_options.is_background = true;
  background_matcher_init_options.matcher_name = background_matcher_method_;
  CHECK(foreground_matcher_->Init(foreground_matcher_init_options));
  CHECK(background_matcher_->Init(background_matcher_init_options));

  // init hm multi target tracker params
  use_histogram_for_match_ = config.use_histogram_for_match();
  histogram_bin_size_ = config.histogram_bin_size();

  is_first_time_ = true;
  return true;
}

bool HmMultiTargetTracker::Track(const MultiTargetTrackerOptions& options,
                                 LidarFrame* frame) {
  // 1. add global offset to pose
  if (is_first_time_) {
    global_to_local_offset_ = -frame->lidar2world_pose.translation();
    last_timestamp_ = current_timestamp_ = frame->timestamp;
  } else {
    last_timestamp_ = current_timestamp_;
    current_timestamp_ = frame->timestamp;
  }
  sensor_to_local_pose_ = frame->lidar2world_pose;
  sensor_to_local_pose_.pretranslate(global_to_local_offset_);
  // 2. transform objects to tracked_objects
  std::vector<TrackedObjectPtr> tracked_objects;
  TrackedObjectPool::Instance().BatchGet(frame->segmented_objects.size(),
                                         &tracked_objects);

  // construct tracked object and do transformation
  std::vector<base::ObjectPtr>& objects = frame->segmented_objects;
  ConstructTrackedObjects(objects, &tracked_objects, sensor_to_local_pose_,
                          options);
  // foreground background split
  DecomposeForegroundBackgroundObjects(tracked_objects, &foreground_objects_,
                                       &background_objects_);

  // 2. update tracks
  if (is_first_time_) {
    LOG_INFO << "..................start track.......................";
    if (!InitializeTracks(options)) {
      LOG_INFO << "Failed to initialize tracks.";
    }
    is_first_time_ = false;
  } else if (!UpdateTracks(options)) {
    LOG_INFO << "Failed to update tracks.";
  }

  // 3. collect result from tracks
  CollectTrackedResult(frame);
  // 4. remove stale track data
  RemoveStaleTrackData();
  return true;
}

bool HmMultiTargetTracker::InitializeTracks(
    const MultiTargetTrackerOptions& options) {
  std::vector<size_t> indices;
  // 1. create new tracks for all foreground objects
  indices.resize(foreground_objects_.size());
  std::iota(indices.begin(), indices.end(), 0);
  CreateNewTracks(foreground_objects_, indices, &foreground_track_data_);
  // 2. create new tracks for all foreground objects
  indices.resize(background_objects_.size());
  std::iota(indices.begin(), indices.end(), 0);
  CreateNewTracks(background_objects_, indices, &background_track_data_);
  return true;
}

bool HmMultiTargetTracker::UpdateTracks(
    const MultiTargetTrackerOptions& options) {
  LOG_INFO << "..................new frame.......................";
  // 1. compute prediction from tracks
  std::vector<Eigen::VectorXf> foreground_track_predicts;
  std::vector<Eigen::VectorXf> background_track_predicts(
      background_track_data_.size(), Eigen::VectorXf::Zero(6));

  double time_diff = current_timestamp_ - last_timestamp_;

  LOG_INFO << "...........fore ground track start................";

  ComputeTracksPredict(foreground_track_data_, &foreground_track_predicts,
                       current_timestamp_);

  std::vector<TrackObjectPair> assignments;
  std::vector<size_t> unassigned_objects;
  std::vector<size_t> unassigned_tracks;
  ObjectTrackMatcherOptions matcher_options;
  // 2. foreground object track association and update
  MatchWrapper(matcher_options, foreground_objects_, foreground_track_data_,
               foreground_track_predicts, time_diff, foreground_matcher_.get(),
               &assignments, &unassigned_tracks, &unassigned_objects);
  LOG_INFO << "Foreground association, #track: "
           << foreground_track_data_.size()
           << " #object: " << foreground_objects_.size()
           << " #assoc: " << assignments.size();

  UpdateAssignedAndUnassignedTracks(foreground_track_data_, foreground_objects_,
                                    assignments, unassigned_tracks,
                                    current_timestamp_);
  // 2.3 create new tracks for non-assigned objects
  CreateNewTracks(foreground_objects_, unassigned_objects,
                  &foreground_track_data_);

  LOG_INFO << "...........back ground track start................";
  // 3. background object track association and update
  MatchWrapper(matcher_options, background_objects_, background_track_data_,
               background_track_predicts, time_diff, background_matcher_.get(),
               &assignments, &unassigned_tracks, &unassigned_objects);
  LOG_INFO << "Background association, #track: "
           << background_track_data_.size()
           << " #object: " << background_objects_.size()
           << " #assoc: " << assignments.size();
  // 3.1 update tracks with assignments
  UpdateAssignedAndUnassignedTracks(background_track_data_, background_objects_,
                                    assignments, unassigned_tracks,
                                    current_timestamp_);
  // 3.3 create new tracks for non-assigned objects
  CreateNewTracks(background_objects_, unassigned_objects,
                  &background_track_data_);

  return true;
}

void HmMultiTargetTracker::ComputeTracksPredict(
    const std::vector<TrackDataPtr>& tracks,
    std::vector<Eigen::VectorXf>* tracks_predict, double timestamp) {
  // Compute tracks' predicted states
  tracks_predict->resize(tracks.size());

  for (size_t i = 0; i < tracks.size(); ++i) {
    (*tracks_predict)[i] =
        (tracker_->Predict(tracks[i], timestamp)).cast<float>();
  }
}

void HmMultiTargetTracker::CollectTrackedResult(LidarFrame* frame) {
  frame->tracked_objects.clear();
  size_t tracked_objects_size =
      foreground_objects_.size() + background_objects_.size();
  size_t foreground_obj_size = foreground_objects_.size();
  frame->tracked_objects.resize(tracked_objects_size);
  std::map<int, int> objects_id_map;
  int track_number = 0;
  std::vector<base::ObjectPtr> tmp_objects;
  base::ObjectPool::Instance().BatchGet(tracked_objects_size, &tmp_objects);

  /* foreground */
  for (size_t i = 0; i < foreground_objects_.size(); i++) {
    TrackedObjectPtr result_obj = foreground_objects_[i];
    base::ObjectPtr obj = tmp_objects[i];
    result_obj->ToObject(obj);

    // cordinate translation
    obj->center -= global_to_local_offset_;
    obj->anchor_point -= global_to_local_offset_;
    for (size_t j = 0; j < (obj->lidar_supplement).cloud_world.size(); ++j) {
      obj->lidar_supplement.cloud_world[j].x -= global_to_local_offset_[0];
      obj->lidar_supplement.cloud_world[j].y -= global_to_local_offset_[1];
      obj->lidar_supplement.cloud_world[j].z -= global_to_local_offset_[2];
    }
    // calculate polygon based on cloud with world coordinates
    hull_.GetConvexHull(obj->lidar_supplement.cloud_world, &obj->polygon);
    ComputeObjectShapeFromPolygon(obj, true);
    int obj_id = obj->id;
    objects_id_map[obj_id] = track_number;
    (frame->tracked_objects)[track_number] = obj;
    track_number++;
  }
  /* background */
  for (size_t i = 0; i < background_objects_.size(); i++) {
    TrackedObjectPtr result_obj = background_objects_[i];
    base::ObjectPtr obj = tmp_objects[foreground_obj_size + i];
    result_obj->ToObject(obj);

    // should we keep these extra pass as a caution that background not move?
    obj->type = base::ObjectType::UNKNOWN_UNMOVABLE;
    obj->velocity = Eigen::Vector3f::Zero();
    // cordinate translation
    obj->center -= global_to_local_offset_;
    obj->anchor_point -= global_to_local_offset_;
    for (size_t j = 0; j < obj->lidar_supplement.cloud_world.size(); ++j) {
      obj->lidar_supplement.cloud_world[j].x -= global_to_local_offset_[0];
      obj->lidar_supplement.cloud_world[j].y -= global_to_local_offset_[1];
      obj->lidar_supplement.cloud_world[j].z -= global_to_local_offset_[2];
    }
    // calculate polygon based on cloud with world coordinates
    hull_.GetConvexHull(obj->lidar_supplement.cloud_world, &obj->polygon);
    ComputeObjectShapeFromPolygon(obj, true);
    int obj_id = obj->id;
    objects_id_map[obj_id] = track_number;
    ((frame->tracked_objects)[track_number]) = obj;
    track_number++;
  }
}

void HmMultiTargetTracker::RemoveStaleTrackData() {
  size_t foreground_track_size = foreground_track_data_.size();
  size_t background_track_size = background_track_data_.size();
  size_t output_foreground_track_num = 0;
  for (size_t i = 0; i < foreground_track_size; ++i) {
    if (IsNeedToRemove(foreground_track_data_[i])) {
      continue;
    }
    foreground_track_data_[output_foreground_track_num++] =
        foreground_track_data_[i];
  }
  foreground_track_data_.resize(output_foreground_track_num);

  // background
  size_t output_background_track_num = 0;
  for (size_t i = 0; i < background_track_size; ++i) {
    if (IsNeedToRemove(background_track_data_[i])) {
      continue;
    }
    background_track_data_[output_background_track_num++] =
        background_track_data_[i];
  }
  background_track_data_.resize(output_background_track_num);
}

bool HmMultiTargetTracker::IsNeedToRemove(const TrackDataPtr& track) {
  if (track->consecutive_invisible_count_ >
      s_maximum_consecutive_invisible_count_) {
    return true;
  }
  float visible_ratio = track->total_visible_count_ * 1.0f / track->age_;
  if (visible_ratio < s_minimum_visible_ratio_) {
    return true;
  }
  return false;
}

void HmMultiTargetTracker::DecomposeForegroundBackgroundObjects(
    const std::vector<TrackedObjectPtr>& tracked_objects,
    std::vector<TrackedObjectPtr>* foreground_objects,
    std::vector<TrackedObjectPtr>* background_objects) {
  foreground_objects->clear();
  foreground_objects->reserve(tracked_objects.size());
  background_objects->clear();
  background_objects->reserve(tracked_objects.size());
  for (auto& object : tracked_objects) {
    if (object->object_ptr->lidar_supplement.is_background) {
      background_objects->push_back(object);
    } else {
      foreground_objects->push_back(object);
    }
  }
}

void HmMultiTargetTracker::MatchWrapper(
    const ObjectTrackMatcherOptions& options,
    std::vector<TrackedObjectPtr>& objects,
    const std::vector<TrackDataPtr>& tracks,
    const std::vector<Eigen::VectorXf>& tracks_predict, const double time_diff,
    ObjectTrackMatcher* matcher, std::vector<TrackObjectPair>* assignments,
    std::vector<size_t>* unassigned_tracks,
    std::vector<size_t>* unassigned_objects) {
  assignments->clear();
  unassigned_tracks->clear();
  unassigned_objects->clear();
  matcher->Match(options, objects, tracks, tracks_predict, time_diff,
                 assignments, unassigned_tracks, unassigned_objects);
}

void HmMultiTargetTracker::CreateNewTracks(
    const std::vector<TrackedObjectPtr>& objects,
    const std::vector<size_t>& unassigned_indices,
    std::vector<TrackDataPtr>* track_datas) {
  LOG_INFO << "new track size " << unassigned_indices.size();
  for (size_t i = 0; i < unassigned_indices.size(); ++i) {
    TrackDataPtr track_data = TrackDataPool::Instance().Get();
    tracker_->UpdateTrackDataWithObject(
        track_data, objects[unassigned_indices[i]], current_timestamp_);
    track_datas->push_back(track_data);
    // LOG_INFO  << "new track " <<
    // objects[unassigned_indices[i]]->belief_anchor_point[0] << " "
    //           << objects[unassigned_indices[i]]->belief_anchor_point[1] << "
    //           "
    //           << objects[unassigned_indices[i]]->belief_anchor_point[2];
  }
}

void HmMultiTargetTracker::ConstructTrackedObjects(
    const std::vector<base::ObjectPtr>& objects,
    std::vector<TrackedObjectPtr>* tracked_objects, const Eigen::Affine3d& pose,
    const MultiTargetTrackerOptions& options) {
  // Construct tracked objects via necessary transformation & feature computing
  int num_objects = objects.size();
  CHECK(objects.size() == tracked_objects->size());
  for (int i = 0; i < num_objects; ++i) {
    ((*tracked_objects)[i])->AttachObject(objects[i], pose);
    // compute foreground objects' shape feature
    if (!objects[i]->lidar_supplement.is_background &&
        use_histogram_for_match_) {
      (*tracked_objects)[i]->histogram_bin_size = histogram_bin_size_;
      (*tracked_objects)[i]->ComputeShapeFeatures();
    }
  }
}

void HmMultiTargetTracker::UpdateAssignedAndUnassignedTracks(
    const std::vector<TrackDataPtr>& tracks,
    const std::vector<TrackedObjectPtr>& new_objects,
    const std::vector<TrackObjectPair>& assignments,
    const std::vector<size_t>& unassigned_tracks, const double timestamp) {
  // Update assigned tracks
  // std::vector<TrackDataPtr>& tracks = ;
  // std::map<size_t, size_t> track_map;
  // for (int i = 0; i < tracks.size(); ++i) {
  //  track_map.insert(std::pair<size_t, size_t>(
  //          tracks[i]->track_id_, i));
  // }

  for (size_t i = 0; i < assignments.size(); i++) {
    size_t track_id = assignments[i].first;
    size_t obj_id = assignments[i].second;
    tracker_->UpdateTrackDataWithObject(tracks[track_id], new_objects[obj_id],
                                        timestamp);
  }

  for (size_t i = 0; i < unassigned_tracks.size(); i++) {
    size_t track_id = unassigned_tracks[i];
    tracker_->UpdateTrackDataWithoutObject(tracks[track_id], timestamp);
  }
}

PERCEPTION_REGISTER_MULTITARGET_TRACKER(HmMultiTargetTracker);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
