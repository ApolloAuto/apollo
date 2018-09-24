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
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/io/file_util.h"
#include "modules/perception/lib/io/protobuf_util.h"

#include "modules/perception/common/graph/secure_matrix.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/gnn_bipartite_graph_matcher.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/multi_hm_bipartite_graph_matcher.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/object_track_matcher.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/proto/hm_tracker_config.pb.h"
#include "object_track_matcher.h"

#include <sstream>

namespace apollo {
namespace perception {
namespace lidar {

bool ObjectTrackMatcher::Init(const ObjectTrackMatcherInitOptions &options) {
  lib::ConfigManager *config_manager =
      lib::Singleton<lib::ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager);
  const lib::ModelConfig *model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = lib::FileUtil::GetAbsolutePath(work_root, root_path);
  config_file =
      lib::FileUtil::GetAbsolutePath(config_file, "object_track_matcher.conf");
  ObjectTrackMatcherConfig config;
  CHECK(lib::ParseProtobufFromFile(config_file, &config));

  track_object_distance_.reset((new TrackObjectDistance()));

  // add params
  float max_match_dist = 4.0;
  float bound_value = 400.0;
  float weight_location_dist = 0.6;
  float weight_direction_dist = 0.2f;
  float weight_bbox_size_dist = 0.1f;
  float weight_point_num_dist = 0.1f;
  float weight_histogram_dist = 0.5f;
  float weight_centroid_shift_dist = 0.2f;
  float weight_bbox_iou_dist = 0.8;
  float background_object_match_threshold = 4.0f;
  max_match_dist = config.max_match_dist();
  if (options.is_background) {
    weight_location_dist = config.bg_weight_location_dist();
    weight_direction_dist = config.bg_weight_direction_dist();
    weight_bbox_size_dist = config.bg_weight_bbox_size_dist();
    weight_point_num_dist = config.bg_weight_point_num_dist();
    weight_histogram_dist = config.bg_weight_histogram_dist();
    weight_centroid_shift_dist = config.bg_weight_centroid_shift_dist();
    weight_bbox_iou_dist = config.bg_weight_bbox_iou_dist();
    background_object_match_threshold =
        config.bg_background_object_match_threshold();
  } else {
    weight_location_dist = config.fg_weight_location_dist();
    weight_direction_dist = config.fg_weight_direction_dist();
    weight_bbox_size_dist = config.fg_weight_bbox_size_dist();
    weight_point_num_dist = config.fg_weight_point_num_dist();
    weight_histogram_dist = config.fg_weight_histogram_dist();
    weight_centroid_shift_dist = config.fg_weight_centroid_shift_dist();
    weight_bbox_iou_dist = config.fg_weight_bbox_iou_dist();
    bound_value = config.bound_value();
  }

  CHECK(set_max_match_distance(max_match_dist));
  CHECK(set_bound_value(bound_value));
  CHECK(track_object_distance_->set_weight_location_dist(weight_location_dist));
  CHECK(
      track_object_distance_->set_weight_direction_dist(weight_direction_dist));
  CHECK(
      track_object_distance_->set_weight_bbox_size_dist(weight_bbox_size_dist));
  CHECK(
      track_object_distance_->set_weight_point_num_dist(weight_point_num_dist));
  CHECK(
      track_object_distance_->set_weight_histogram_dist(weight_histogram_dist));
  CHECK(track_object_distance_->set_weight_centroid_shift_dist(
      weight_centroid_shift_dist));
  CHECK(track_object_distance_->set_weight_bbox_iou_dist(weight_bbox_iou_dist));
  CHECK(track_object_distance_->set_background_object_match_threshold(
      background_object_match_threshold));

  matcher_.reset(BaseBipartiteGraphMatcherRegisterer::GetInstanceByName(
      options.matcher_name));
  CHECK(matcher_ != nullptr) << " failed to get " << options.matcher_name;
  return true;
}

bool ObjectTrackMatcher::set_max_match_distance(float max_match_distance) {
  CHECK_GE(max_match_distance, 0);
  max_match_distance_ = max_match_distance;
  return true;
}

bool ObjectTrackMatcher::set_bound_value(float bound_value) {
  CHECK_GE(bound_value, 0);
  bound_value_ = bound_value;
  return true;
}
void ObjectTrackMatcher::Match(
    const ObjectTrackMatcherOptions &options,
    std::vector<TrackedObjectPtr> &objects,
    const std::vector<TrackDataPtr> &tracks,
    const std::vector<Eigen::VectorXf> &tracks_predict, const double time_diff,
    std::vector<TrackObjectPair> *assignments,
    std::vector<size_t> *unassigned_tracks,
    std::vector<size_t> *unassigned_objects) {
  assignments->clear();
  unassigned_objects->clear();
  unassigned_tracks->clear();
  if (objects.size() == 0 || tracks.size() == 0) {
    unassigned_objects->resize(objects.size());
    unassigned_tracks->resize(tracks.size());
    for (size_t i = 0; i < objects.size(); ++i) {
      unassigned_objects->at(i) = i;
    }
    for (size_t i = 0; i < tracks.size(); ++i) {
      unassigned_tracks->at(i) = i;
    }
    return;
  }

  BipartiteGraphMatcherOptions matcher_options;
  matcher_options.cost_thresh = max_match_distance_;
  matcher_options.bound_value = bound_value_;
  common::SecureMat<float> *association_mat = matcher_->cost_matrix();
  association_mat->reserve(1000, 1000);
  // 1. computing association matrix
  association_mat->resize(tracks.size(), objects.size());
  ComputeAssociateMatrix(tracks, tracks_predict, objects, time_diff,
                         association_mat);
  matcher_->Match(matcher_options, assignments, unassigned_tracks,
                  unassigned_objects);
  for (size_t i = 0; i < assignments->size(); ++i) {
    objects[assignments->at(i).second]->association_score =
        (*association_mat)(assignments->at(i).first,
                           assignments->at(i).second) /
        max_match_distance_;
  }
}

void ObjectTrackMatcher::ComputeAssociateMatrix(
    const std::vector<TrackDataPtr> &tracks,
    const std::vector<Eigen::VectorXf> &tracks_predict,
    const std::vector<TrackedObjectPtr> &new_objects, const double time_diff,
    common::SecureMat<float> *association_mat) {
  // Compute matrix of association distance
  for (size_t i = 0; i < tracks.size(); ++i) {
    for (size_t j = 0; j < new_objects.size(); ++j) {
      (*association_mat)(i, j) = ComputeTrackObjectDistance(
          tracks[i], tracks_predict[i], new_objects[j], time_diff);
    }
  }

  /*
  for (size_t i = 0; i < tracks.size(); ++i) {
    std::stringstream ss;
    for (size_t j = 0; j < new_objects.size(); ++j) {
      ss << (*association_mat)(i, j) << " ";
    }
    LOG_INFO << "association matrix : "<< ss.str();
  }*/
}

float ObjectTrackMatcher::ComputeTrackObjectDistance(
    const TrackDataPtr &track, const Eigen::VectorXf &track_predict,
    const TrackedObjectPtr &new_object, const double time_diff) {
  // Compute distance of given track & object
  return track_object_distance_->ComputeDistance(track, track_predict,
                                                 new_object, time_diff);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
