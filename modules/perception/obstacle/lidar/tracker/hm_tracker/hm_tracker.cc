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

#include <map>
#include <vector>
#include "modules/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/feature_descriptor.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/hungarian_matcher.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/kalman_filter.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/track_object_distance.h"

namespace apollo {
namespace perception {

HmObjectTracker::HmObjectTracker(): matcher_method_(HUNGARIAN_MATCHER),
  filter_method_(KALMAN_FILTER),
  use_histogram_for_match_(false),
  histogram_bin_size_(10),
  matcher_(NULL),
  time_stamp_(0.0),
  valid_(false) {
}

HmObjectTracker::~HmObjectTracker() {
  if (matcher_) {
    delete matcher_;
    matcher_ = NULL;
  }
}

bool HmObjectTracker::Init() {
  // Initialize tracker's configs
  using apollo::perception::ConfigManager;
  using apollo::perception::ModelConfig;

  const ModelConfig* model_config = NULL;
  if (!ConfigManager::instance()->GetModelConfig(name(), &model_config)) {
    AERROR << "not found model config: " << name();
    return false;
  }

  // A. Tracker setup
  std::string matcher_method_name = "hungarian_matcher";
  if (!model_config->GetValue("matcher_method_name",
    &matcher_method_name)) {
    AERROR << "matcher_method_name not found." << name();
    return false;
  }
  if (matcher_method_name == "hungarian_matcher") {
    matcher_method_ = HUNGARIAN_MATCHER;
    matcher_ = new HungarianMatcher();
  } else {
    AERROR << "Invalid matcher_method_name " << name();
    return false;
  }

  std::string filter_method_name = "kalman_filter";
  if (!model_config->GetValue("filter_method_name", &filter_method_name)) {
    AERROR << "filter_method_name not found." << name();
    return false;
  }
  if (filter_method_name == "kalman_filter") {
    filter_method_ = KALMAN_FILTER;
    ObjectTrack::SetFilterMethod(filter_method_);
  } else {
    AERROR << "Invalid filter_method_name " << name();
    return false;
  }

  if (!model_config->GetValue("consecutive_invisible_count_minimum",
    &consecutive_invisible_count_minimum_)) {
    AERROR << "Failed to get consecutive invisible count minimum! "
      << name();
    return false;
  }
  if (!model_config->GetValue("collect_age_minimum",
    &collect_age_minimum_)) {
    AERROR << "Failed to get collect age minimum! " << name();
    return false;
  }

  int invisible_window = 1;
  if (!model_config->GetValue("invisible_window",
    &invisible_window)) {
    AERROR << "Failed to get invisible window! " << name();
    return false;
  }
  ObjectTrackSet::s_maximum_consecutive_invisible_count_ = invisible_window;

  // B. Matcher setup
  float max_match_dist = 4.0;
  float weight_location_dist = 0.6;
  float weight_direction_dist = 0.2f;
  float weight_bbox_size_dist = 0.1f;
  float weight_point_num_dist = 0.1f;
  float weight_histogram_dist = 0.5f;

  if (!model_config->GetValue("max_match_distance", &max_match_dist)) {
    AERROR << "max_match_distance not found. " << name();
    return false;
  }
  if (matcher_method_ == HUNGARIAN_MATCHER) {
    if (!HungarianMatcher::SetMaxMatchDistance(max_match_dist)) {
      return false;
    }
  }

  if (!model_config->GetValue("weight_location_dist",
    &weight_location_dist)) {
    AERROR << "weight_location_dist not found." << name();
    return false;
  }
  if (!TrackObjectDistance::SetWeightLocationDist(weight_location_dist)) {
    return false;
  }

  if (!model_config->GetValue("weight_direction_dist",
    &weight_direction_dist)) {
    AERROR << "weight_direction_dist not found." << name();
    return false;
  }
  if (!TrackObjectDistance::SetWeightDirectionDist(
    weight_direction_dist)) {
    return false;
  }

  if (!model_config->GetValue("weight_bbox_size_dist",
    &weight_bbox_size_dist)) {
    AERROR << "weight_bbox_size_dist not found." << name();
    return false;
  }
  if (!TrackObjectDistance::SetWeightBboxSizeDist(
    weight_bbox_size_dist)) {
    return false;
  }

  if (!model_config->GetValue("weight_point_num_dist",
    &weight_point_num_dist)) {
    AERROR << "weight_point_num_dist not found." << name();
    return false;
  }
  if (!TrackObjectDistance::SetWeightPointNumDist(
    weight_point_num_dist)) {
    return false;
  }

  if (!model_config->GetValue("weight_histogram_dist",
    &weight_histogram_dist)) {
    AERROR << "weight_histogram_dist not found." << name();
    return false;
  }
  if (!TrackObjectDistance::SetWeightHistogramDist(
    weight_histogram_dist)) {
    return false;
  }
  use_histogram_for_match_ = weight_histogram_dist > 0.01 ? true : false;
  if (!model_config->GetValue("histogram_bin_size", &histogram_bin_size_)) {
    AERROR << "histogram_bin_size not found." << name();
    return false;
  }

  // C. Filter setup
  bool use_adaptive = false;
  if (!model_config->GetValue("use_adaptive", &use_adaptive)) {
    AERROR << "use_adaptive not found." << name();
    return false;
  }

  if (filter_method_ == KALMAN_FILTER) {
    KalmanFilter::SetUseAdaptive(use_adaptive);
    double max_adaptive_score = max_match_dist;
    KalmanFilter::SetMaxAdaptiveScore(max_adaptive_score);
    float centroid_measurement_noise = 0.4f;
    float centroid_initial_velocity_variance = 5.0f;
    float propagation_variance_xy = 10.0f;
    float propagation_variance_z = 10.0f;
    if (!model_config->GetValue("centroid_measurement_noise",
      &centroid_measurement_noise)) {
      AERROR << "centroid_measurement_noise not found." << name();
      return false;
    }
    if (!model_config->GetValue("centroid_initial_velocity_variance",
      &centroid_initial_velocity_variance)) {
      AERROR << "centroid_initial_velocity_variance not found." << name();
      return false;
    }
    if (!model_config->GetValue("propagation_variance_xy",
      &propagation_variance_xy)) {
      AERROR << "propagation_variance_xy not found." << name();
      return false;
    }
    if (!model_config->GetValue("propagation_variance_z",
      &propagation_variance_z)) {
      AERROR << "propagation_variance_z not found." << name();
      return false;
    }
    KalmanFilter::InitParams(centroid_measurement_noise,
      centroid_initial_velocity_variance,
      propagation_variance_xy,
      propagation_variance_z);
  }
  return true;
}

const std::vector<ObjectTrackPtr>& HmObjectTracker::GetObjectTracks() const {
  return object_tracks_.get_tracks();
}

Eigen::Matrix4d HmObjectTracker::GetPose() const {
  return velodyne_to_local_pose_;
}

bool HmObjectTracker::Track(const std::vector<ObjectPtr>& objects,
  double timestamp,
  const TrackerOptions& options,
  std::vector<ObjectPtr>* tracked_objects) {
  // Track detected objects over consecutive frames

  /* 0. setup tracker */
  if (tracked_objects == NULL) {
    return false;
  }
  if (!valid_) {
    valid_ = true;
    return Initialize(objects, timestamp, options, tracked_objects);
  }

  Eigen::Matrix4d velo2world_pose = Eigen::Matrix4d::Identity();
  if (options.velodyne_trans != NULL) {
    velo2world_pose = *(options.velodyne_trans);
  } else {
    AWARN << "Input velodyne2world_pose is null\n";
  }
  double time_diff = timestamp - time_stamp_;
  time_stamp_ = timestamp;

  /* 1. pre-processing */
  // 1.1 transform pose's coordinates
  TransformPoseGlobal2Local(&velo2world_pose);
  AINFO << "velo2local_pose \n" << velo2world_pose;

  // 1.2 construct objects for tracking
  std::vector<TrackedObjectPtr> transformed_objects;
  ConstructTrackedObjects(objects, &transformed_objects, velo2world_pose,
                          options);

  // 1.3 decompose foreground & background
  std::vector<TrackedObjectPtr> foreground_objects;
  DecomposeForegroundBackgroundObjects(&transformed_objects,
    &foreground_objects, &background_objects_);
  // 1.4 compute reference
  Eigen::Vector3f cur_location(velo2world_pose(0, 3), velo2world_pose(1, 3),
    velo2world_pose(2, 3));
  ref_translation_ = cur_location - ref_location_;
  ref_location_ = cur_location;
  ref_orientation_ = (velo2world_pose *
    Eigen::Vector4d(1, 0, 0, 0)).head(3).cast<float>();
  velodyne_to_local_pose_ = velo2world_pose;

  AINFO << "object_track_number = " << object_tracks_.size() << " "
    << "object_number = " << objects.size() << "\n";

  /* 2. predict tracks */
  std::vector<Eigen::VectorXf> tracks_predict;
  ComputeTracksPredict(&tracks_predict, time_diff);

  /* 3. match objects & tracks */
  std::vector<TrackObjectPair> assignments;
  std::vector<int> unassigned_objects;
  std::vector<int> unassigned_tracks;
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.get_tracks();

  matcher_->Match(&foreground_objects, tracks, tracks_predict,
    time_diff, &assignments, &unassigned_tracks, &unassigned_objects);

  AINFO << "multi-object-tracking: " << tracks.size() << "  "
    << assignments.size() << "  " << foreground_objects.size() << "  "
    << unassigned_objects.size() << "  " << time_diff << "\n";

  /* 4. update tracks */
  // 4.1 update tracks with associated objects
  UpdateAssignedTracks(&tracks_predict, &foreground_objects, assignments,
    time_diff);

  // 4.2 update tracks without associated objects
  UpdateUnassignedTracks(tracks_predict, unassigned_tracks, time_diff);
  DeleteLostTracks();

  // 4.3 create new tracks for objects without associated tracks
  CreateNewTracks(foreground_objects, unassigned_objects, time_diff);

  /* 5. collect results */
  CollectTrackedResults(tracked_objects);

  ComputeTrackIdsForRecentObjects(transformed_objects);

  // update frame
  valid_ = true;

  return true;
}

bool HmObjectTracker::Initialize(const std::vector<ObjectPtr>& objects,
  double timestamp,
  const TrackerOptions& options,
  std::vector<ObjectPtr>* tracked_objects) {
  /* 0. setup tracker */
  Eigen::Matrix4d velo2world_pose = Eigen::Matrix4d::Identity();
  if (options.velodyne_trans != NULL) {
    velo2world_pose = *(options.velodyne_trans);
  } else {
    AERROR << "Input velodyne2world_pose in HmObjectTracker is null\n";
    return false;
  }
  global_to_local_offset_ = Eigen::Vector3d(-velo2world_pose(0, 3),
    -velo2world_pose(1, 3), -velo2world_pose(2, 3));

  /* 1. pre-processing */
  // 1.1 coordinate transformation
  TransformPoseGlobal2Local(&velo2world_pose);
  AINFO << "velo2local_pose \n" << velo2world_pose;

  // 1.2 construct tracked objects
  std::vector<TrackedObjectPtr> transformed_objects;
  ConstructTrackedObjects(objects, &transformed_objects, velo2world_pose,
                          options);

  // 1.3 decompose foreground & background
  std::vector<TrackedObjectPtr> foreground_objects;
  DecomposeForegroundBackgroundObjects(&transformed_objects,
    &foreground_objects, &background_objects_);

  // 1.4 compute refernce
  ref_location_ = Eigen::Vector3f(velo2world_pose(0, 3),
    velo2world_pose(1, 3), velo2world_pose(2, 3));
  ref_translation_ = Eigen::Vector3f(0, 0, 0);
  Eigen::Vector4d dir = velo2world_pose * Eigen::Vector4d(1, 0, 0, 0);
  ref_orientation_ = Eigen::Vector3f(dir[0], dir[1], dir[2]);

  /* 2. update tracks */
  std::vector<int> unassigned_objects;
  unassigned_objects.resize(foreground_objects.size());
  for (size_t i = 0; i < foreground_objects.size(); i++) {
    unassigned_objects[i] = i;
  }
  double time_diff = 0.1;
  CreateNewTracks(foreground_objects, unassigned_objects, time_diff);
  time_stamp_ = timestamp;
  velodyne_to_local_pose_ = velo2world_pose;

  /* 3. collect results */
  ComputeTrackIdsForRecentObjects(transformed_objects);

  CollectTrackedResults(tracked_objects);

  return true;
}

void HmObjectTracker::TransformPoseGlobal2Local(Eigen::Matrix4d* pose) {
  // Trasnform velodyne2world_pose to velodyne2local_pose to avoid computing
  // big float value in hm tracker
  (*pose)(0, 3) += global_to_local_offset_[0];
  (*pose)(1, 3) += global_to_local_offset_[1];
  (*pose)(2, 3) += global_to_local_offset_[2];
}

void HmObjectTracker::ConstructTrackedObjects(
  const std::vector<ObjectPtr>& objects,
  std::vector<TrackedObjectPtr>* tracked_objects,
  const Eigen::Matrix4d& pose,
  const TrackerOptions& options) {
  // Construct tracked objects. Help tracked objects with necessary
  // transformation & feature extraction & lane direction query
  int num_objects = objects.size();
  tracked_objects->clear();
  tracked_objects->resize(num_objects);
  for (int i = 0; i < num_objects; ++i) {
    ObjectPtr obj(new Object());
    obj->clone(*objects[i]);
    (*tracked_objects)[i].reset(new TrackedObject(obj));
    // Computing shape featrue for foreground objects
    if (!obj->is_background && use_histogram_for_match_) {
      ComputeShapeFeatures(&((*tracked_objects)[i]));
    }
    // Transforming all tracked objects
    TransformTrackedObject(&((*tracked_objects)[i]), pose);
    // Setting barycenter as anchor point of tracked objects
    Eigen::Vector3f anchor_point = (*tracked_objects)[i]->barycenter;
    (*tracked_objects)[i]->anchor_point = anchor_point;
    // Getting lane direction of tracked objects
    pcl_util::PointD query_pt;
    query_pt.x = anchor_point(0) - global_to_local_offset_(0);
    query_pt.y = anchor_point(1) - global_to_local_offset_(1);
    query_pt.z = anchor_point(2) - global_to_local_offset_(2);
    Eigen::Vector3d lane_dir;
    if (!options.hdmap_input->GetNearestLaneDirection(query_pt, &lane_dir)) {
      AERROR << "Failed to initialize the lane direction of tracked object";
      // Set lane dir as host dir if query lane direction failed
      lane_dir = (pose * Eigen::Vector4d(1, 0, 0, 0)).head(3);
    }
    (*tracked_objects)[i]->lane_direction = lane_dir.cast<float>();
  }
}

void HmObjectTracker::ComputeShapeFeatures(TrackedObjectPtr* obj) {
  // Compute object's shape feature
  ObjectPtr& temp_object = (*obj)->object_ptr;
  FeatureDescriptor fd(temp_object->cloud);
  fd.ComputeHistogram(histogram_bin_size_, &temp_object->shape_features);
}

void HmObjectTracker::TransformTrackedObject(TrackedObjectPtr* obj,
  const Eigen::Matrix4d& pose) {
  // Transform tracked object with given pose
  TransformObject(&((*obj)->object_ptr), pose);

  Eigen::Vector3f& dir = (*obj)->direction;
  dir = (pose *
    Eigen::Vector4d(dir[0], dir[1], dir[2], 0)).head(3).cast<float>();

  Eigen::Vector3f& center = (*obj)->center;
  center = (pose * Eigen::Vector4d(
    center[0], center[1], center[2], 1)).head(3).cast<float>();

  Eigen::Vector3f& baryct = (*obj)->barycenter;
  baryct = (pose * Eigen::Vector4d(
    baryct[0], baryct[1], baryct[2], 1)).head(3).cast<float>();
}

void HmObjectTracker::TransformObject(ObjectPtr* obj,
  const Eigen::Matrix4d& pose) {
  // Transform object with given pose
  Eigen::Vector3d& dir = (*obj)->direction;
  dir = (pose * Eigen::Vector4d(dir[0], dir[1], dir[2], 0)).head(3);

  Eigen::Vector3d& center = (*obj)->center;
  center = (pose * Eigen::Vector4d(
    center[0], center[1], center[2], 1)).head(3);

  TransformPointCloud<pcl_util::Point>(pose, (*obj)->cloud);
  TransformPointCloud<pcl_util::PointD>(pose, &((*obj)->polygon));
}

void HmObjectTracker::DecomposeForegroundBackgroundObjects(
  std::vector<TrackedObjectPtr>* objects,
  std::vector<TrackedObjectPtr>* fg_objects,
  std::vector<TrackedObjectPtr>* bg_objects) {
  // Decompose foreground background from detected objects pool
  fg_objects->clear();
  bg_objects->clear();

  for (size_t i = 0; i < objects->size(); i++) {
    if ((*objects)[i]->object_ptr->is_background) {
      (*objects)[i]->object_ptr->track_id = ObjectTrack::GetNextTrackId();
      bg_objects->push_back((*objects)[i]);
    } else {
      fg_objects->push_back((*objects)[i]);
    }
  }
}

void HmObjectTracker::ComputeTracksPredict(
  std::vector<Eigen::VectorXf>* tracks_predict,
  const double time_diff) {
  // Compute tracks' predicted states
  int no_track = object_tracks_.size();
  tracks_predict->resize(no_track);
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.get_tracks();

  for (int i = 0; i < no_track; ++i) {
    (*tracks_predict)[i] = tracks[i]->Predict(time_diff);
  }
}

void HmObjectTracker::UpdateAssignedTracks(
  std::vector<Eigen::VectorXf>* tracks_predict,
  std::vector<TrackedObjectPtr>* new_objects,
  const std::vector<TrackObjectPair>& assignments,
  const double time_diff) {
  // Update assigned tracks
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.get_tracks();

  for (size_t i = 0; i < assignments.size(); i++) {
    int track_id = assignments[i].first;
    int obj_id = assignments[i].second;
    tracks[track_id]->UpdateWithObject(&(*new_objects)[obj_id], time_diff);
  }
}

void HmObjectTracker::UpdateUnassignedTracks(
  const std::vector<Eigen::VectorXf>& tracks_predict,
  const std::vector<int>& unassigned_tracks,
  const double time_diff) {
  // Update tracks without matched objects
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.get_tracks();

  for (size_t i = 0; i < unassigned_tracks.size(); i++) {
    int track_id = unassigned_tracks[i];
    tracks[track_id]->UpdateWithoutObject(
      tracks_predict[track_id], time_diff);
  }
}

void HmObjectTracker::CreateNewTracks(
  const std::vector<TrackedObjectPtr>& new_objects,
  const std::vector<int>& unassigned_objects,
  const double time_diff) {
  // Create new tracks for objects without matched tracks
  for (size_t i = 0; i < unassigned_objects.size(); i++) {
    int obj_id = unassigned_objects[i];
    ObjectTrackPtr track(new ObjectTrack(new_objects[obj_id]));
    object_tracks_.add_track(track);
  }
}

void HmObjectTracker::DeleteLostTracks() {
  // Delete lost tracks
  object_tracks_.remove_lost_tracks();
}

void HmObjectTracker::CollectTrackedResults(
  std::vector<ObjectPtr>* tracked_objects) {
  // Collect tracked results for reporting include objects may be occluded
  // temporaryly
  const std::vector<ObjectTrackPtr>& tracks = object_tracks_.get_tracks();
  tracked_objects->resize(tracks.size() + background_objects_.size());

  int track_number = 0;
  for (size_t i = 0; i < tracks.size(); i++) {
    if (tracks[i]->consecutive_invisible_count_ >
      consecutive_invisible_count_minimum_)
      continue;
    if (tracks[i]->age_ < collect_age_minimum_)
      continue;
    ObjectPtr obj(new Object);
    TrackedObjectPtr result_obj = tracks[i]->current_object_;

    obj->clone(*(result_obj->object_ptr));

    obj->direction = result_obj->direction.cast<double>();
    if (fabs(obj->direction[0]) < DBL_MIN) {
      if (obj->direction[1] > 0) {
        obj->theta = M_PI / 2;
      } else {
        obj->theta = -M_PI / 2;
      }
    } else {
      obj->theta = atan2(obj->direction[1], obj->direction[0]);
    }

    obj->length = result_obj->size[0];
    obj->width = result_obj->size[1];
    obj->height = result_obj->size[2];
    obj->velocity = result_obj->velocity.cast<double>();
    obj->track_id = tracks[i]->idx_;
    obj->tracking_time = tracks[i]->period_;
    obj->type = result_obj->type;
    obj->center = result_obj->center.cast<double>() - global_to_local_offset_;
    obj->anchor_point = result_obj->anchor_point.cast<double>() -
      global_to_local_offset_;
    for (size_t j = 0; j < obj->cloud->size(); j++) {
      obj->cloud->points[j].x -= global_to_local_offset_[0];
      obj->cloud->points[j].y -= global_to_local_offset_[1];
      obj->cloud->points[j].z -= global_to_local_offset_[2];
    }
    for (size_t j = 0; j < obj->polygon.size(); j++) {
      obj->polygon.points[j].x -= global_to_local_offset_[0];
      obj->polygon.points[j].y -= global_to_local_offset_[1];
      obj->polygon.points[j].z -= global_to_local_offset_[2];
    }

    (*tracked_objects)[track_number] = obj;
    track_number++;
  }

  for (size_t i = 0; i < background_objects_.size(); i++) {
    ObjectPtr obj(new Object);
    obj->clone(*(background_objects_[i]->object_ptr));

    if (fabs(obj->direction[0]) < DBL_MIN) {
      if (obj->direction[1] > 0) {
        obj->theta = M_PI / 2;
      } else {
        obj->theta = -M_PI / 2;
      }
    } else {
      obj->theta = atan2(obj->direction[1], obj->direction[0]);
    }

    obj->tracking_time = 0;

    obj->center -= global_to_local_offset_;
    obj->anchor_point = obj->center;
    for (size_t j = 0; j < obj->cloud->size(); j++) {
      obj->cloud->points[j].x -= global_to_local_offset_[0];
      obj->cloud->points[j].y -= global_to_local_offset_[1];
      obj->cloud->points[j].z -= global_to_local_offset_[2];
    }
    for (size_t j = 0; j < obj->polygon.size(); j++) {
      obj->polygon.points[j].x -= global_to_local_offset_[0];
      obj->polygon.points[j].y -= global_to_local_offset_[1];
      obj->polygon.points[j].z -= global_to_local_offset_[2];
    }

    (*tracked_objects)[track_number] = obj;
    track_number++;
  }
  tracked_objects->resize(track_number);
}

void HmObjectTracker::ComputeTrackIdsForRecentObjects(
  const std::vector<TrackedObjectPtr>& objects) {
  // Compute track ids for recent objects
  std::map<int, int> recent_obj_2_track;
  const std::vector<ObjectTrackPtr>& tracks = object_tracks_.get_tracks();
  for (size_t i = 0; i < tracks.size(); i++) {
    if (tracks[i]->consecutive_invisible_count_ == 0) {
      int obj_id = tracks[i]->current_object_->object_ptr->id;
      recent_obj_2_track[obj_id] = tracks[i]->idx_;
    }
  }

  for (size_t i = 0; i < background_objects_.size(); i++) {
    int obj_id = background_objects_[i]->object_ptr->id;
    recent_obj_2_track[obj_id] = background_objects_[i]->object_ptr->track_id;
  }

  track_ids_for_recent_objects_.assign(objects.size(), -1);
  for (size_t i = 0; i < objects.size(); i++) {
    int obj_id = objects[i]->object_ptr->id;
    std::map<int, int>::iterator it = recent_obj_2_track.find(obj_id);
    if (it != recent_obj_2_track.end()) {
      track_ids_for_recent_objects_[i] = it->second;
    }
  }
}

}  // namespace perception
}  // namespace apollo
