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

#include "modules/perception/obstacle/lidar/tracker/hm_tracker/hungarian_matcher.h"

#include "modules/common/log.h"
#include "modules/perception/common/geometry_util.h"
#include "modules/perception/common/graph_util.h"
#include "modules/perception/obstacle/common/hungarian_bigraph_matcher.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/track_object_distance.h"

namespace apollo {
namespace perception {

float HungarianMatcher::s_match_distance_maximum_ = 4.0f;

bool HungarianMatcher::SetMatchDistanceMaximum(
    const float& match_distance_maximum) {
  if (match_distance_maximum >= 0) {
    s_match_distance_maximum_ = match_distance_maximum;
    AINFO << "match distance maximum of HungarianMatcher is "
          << s_match_distance_maximum_;
    return true;
  }
  AERROR << "invalid match distance maximum of HungarianMatcher!";
  return false;
}

void HungarianMatcher::Match(
    std::vector<std::shared_ptr<TrackedObject>>* objects,
    const std::vector<ObjectTrackPtr>& tracks,
    const std::vector<Eigen::VectorXf>& tracks_predict,
    std::vector<std::pair<int, int>>* assignments,
    std::vector<int>* unassigned_tracks, std::vector<int>* unassigned_objects) {
  // A. computing association matrix
  Eigen::MatrixXf association_mat(tracks.size(), objects->size());
  ComputeAssociateMatrix(tracks, tracks_predict, (*objects), &association_mat);

  // B. computing connected components
  std::vector<std::vector<int>> object_components;
  std::vector<std::vector<int>> track_components;
  ComputeConnectedComponents(association_mat, s_match_distance_maximum_,
                             &track_components, &object_components);
  ADEBUG << "HungarianMatcher: partition graph into " << track_components.size()
         << " sub-graphs.";

  // C. matching each sub-graph
  assignments->clear();
  unassigned_tracks->clear();
  unassigned_objects->clear();
  for (size_t i = 0; i < track_components.size(); i++) {
    std::vector<std::pair<int, int>> sub_assignments;
    std::vector<int> sub_unassigned_tracks;
    std::vector<int> sub_unassigned_objects;
    MatchInComponents(association_mat, track_components[i],
                      object_components[i], &sub_assignments,
                      &sub_unassigned_tracks, &sub_unassigned_objects);
    for (size_t j = 0; j < sub_assignments.size(); ++j) {
      int track_id = sub_assignments[j].first;
      int object_id = sub_assignments[j].second;
      assignments->push_back(sub_assignments[j]);
      float association_score = association_mat(track_id, object_id);
      (*objects)[object_id]->association_score = association_score;
    }
    for (size_t j = 0; j < sub_unassigned_tracks.size(); ++j) {
      unassigned_tracks->push_back(sub_unassigned_tracks[j]);
    }
    for (size_t j = 0; j < sub_unassigned_objects.size(); ++j) {
      unassigned_objects->push_back(sub_unassigned_objects[j]);
    }
  }
}

void HungarianMatcher::MatchInComponents(
    const Eigen::MatrixXf& association_mat,
    const std::vector<int>& track_component,
    const std::vector<int>& object_component,
    std::vector<std::pair<int, int>>* sub_assignments,
    std::vector<int>* sub_unassigned_tracks,
    std::vector<int>* sub_unassigned_objects) {
  sub_assignments->clear();
  sub_unassigned_tracks->clear();
  sub_unassigned_objects->clear();
  // A. failted to match if either components is empty
  if (track_component.empty()) {
    for (size_t i = 0; i < object_component.size(); ++i) {
      sub_unassigned_objects->push_back(object_component[i]);
    }
  }
  if (object_component.empty()) {
    for (size_t i = 0; i < track_component.size(); ++i) {
      sub_unassigned_tracks->push_back(track_component[i]);
    }
  }
  if (track_component.empty() || object_component.empty()) return;
  // B. if components perfectly match
  if (track_component.size() == 1 && object_component.size() == 1) {
    int track_id = track_component[0];
    int object_id = object_component[0];
    if (association_mat(track_id, object_id) <= s_match_distance_maximum_) {
      sub_assignments->push_back(std::make_pair(track_id, object_id));
    } else {
      sub_unassigned_objects->push_back(object_id);
      sub_unassigned_tracks->push_back(track_id);
    }
    return;
  }
  // C. multi object track match
  std::vector<int> track_local2global;
  std::vector<int> object_local2global;
  std::vector<std::pair<int, int>> local_assignments;
  std::vector<int> local_unassigned_tracks;
  std::vector<int> local_unassigned_objects;
  Eigen::MatrixXf local_association_mat(track_component.size(),
                                        object_component.size());
  track_local2global.resize(track_component.size());
  object_local2global.resize(object_component.size());
  for (size_t i = 0; i < track_component.size(); ++i) {
    track_local2global[i] = track_component[i];
  }
  for (size_t i = 0; i < object_component.size(); ++i) {
    object_local2global[i] = object_component[i];
  }
  for (size_t i = 0; i < track_component.size(); ++i) {
    for (size_t j = 0; j < object_component.size(); ++j) {
      int track_id = track_component[i];
      int object_id = object_component[j];
      local_association_mat(i, j) = association_mat(track_id, object_id);
    }
  }
  local_assignments.resize(local_association_mat.cols());
  local_unassigned_tracks.assign(local_association_mat.rows(), -1);
  local_unassigned_objects.assign(local_association_mat.cols(), -1);
  AssignObjectsToTracks(local_association_mat, s_match_distance_maximum_,
                        &local_assignments, &local_unassigned_tracks,
                        &local_unassigned_objects);
  for (size_t i = 0; i < local_assignments.size(); ++i) {
    int global_track_id = track_local2global[local_assignments[i].first];
    int global_object_id = object_local2global[local_assignments[i].second];
    sub_assignments->push_back(
        std::make_pair(global_track_id, global_object_id));
  }
  for (size_t i = 0; i < local_unassigned_tracks.size(); ++i) {
    int global_track_id = track_local2global[local_unassigned_tracks[i]];
    sub_unassigned_tracks->push_back(global_track_id);
  }
  for (size_t i = 0; i < local_unassigned_objects.size(); ++i) {
    int global_object_id = object_local2global[local_unassigned_objects[i]];
    sub_unassigned_objects->push_back(global_object_id);
  }
}

void HungarianMatcher::ComputeAssociateMatrix(
    const std::vector<ObjectTrackPtr>& tracks,
    const std::vector<Eigen::VectorXf>& tracks_predict,
    const std::vector<std::shared_ptr<TrackedObject>>& new_objects,
    Eigen::MatrixXf* association_mat) {
  // Compute matrix of association distance
  for (size_t i = 0; i < tracks.size(); ++i) {
    for (size_t j = 0; j < new_objects.size(); ++j) {
      (*association_mat)(i, j) = TrackObjectDistance::ComputeDistance(
          tracks[i], tracks_predict[i], new_objects[j]);
    }
  }
}

void HungarianMatcher::ComputeConnectedComponents(
    const Eigen::MatrixXf& association_mat, const float& connected_threshold,
    std::vector<std::vector<int>>* track_components,
    std::vector<std::vector<int>>* object_components) {
  // Compute connected components within given threshold
  int no_track = association_mat.rows();
  int no_object = association_mat.cols();
  std::vector<std::vector<int>> nb_graph;
  nb_graph.resize(no_track + no_object);
  for (int i = 0; i < no_track; i++) {
    for (int j = 0; j < no_object; j++) {
      if (association_mat(i, j) <= connected_threshold) {
        nb_graph[i].push_back(no_track + j);
        nb_graph[j + no_track].push_back(i);
      }
    }
  }

  std::vector<std::vector<int>> components;
  ConnectedComponentAnalysis(nb_graph, &components);
  track_components->clear();
  track_components->resize(components.size());
  object_components->clear();
  object_components->resize(components.size());
  for (size_t i = 0; i < components.size(); i++) {
    for (size_t j = 0; j < components[i].size(); j++) {
      int id = components[i][j];
      if (id < no_track) {
        (*track_components)[i].push_back(id);
      } else {
        id -= no_track;
        (*object_components)[i].push_back(id);
      }
    }
  }
}

void HungarianMatcher::AssignObjectsToTracks(
    const Eigen::MatrixXf& association_mat,
    const double& assign_distance_maximum,
    std::vector<std::pair<int, int>>* assignments,
    std::vector<int>* unassigned_tracks, std::vector<int>* unassigned_objects) {
  // Assign objects to tracks with null tracks setup
  std::vector<int> tracks_idx;
  std::vector<int> objects_idx;
  int no_track = association_mat.rows();
  int no_object = association_mat.cols();
  // build cost
  std::vector<std::vector<double>> cost(no_track + no_object);
  for (int i = 0; i < no_track; ++i) {
    cost[i].resize(association_mat.cols());
    for (int j = 0; j < association_mat.cols(); ++j) {
      cost[i][j] = association_mat(i, j);
    }
  }
  for (int i = 0; i < no_object; ++i) {
    cost[i + no_track].resize(no_object);
    for (int j = 0; j < no_object; ++j) {
      if (j == i) {
        cost[i + no_track][j] = assign_distance_maximum * 1.2f;
      } else {
        cost[i + no_track][j] = 999999.0f;
      }
    }
  }

  HungarianOptimizer hungarian_optimizer(cost);
  hungarian_optimizer.minimize(&tracks_idx, &objects_idx);

  int assignments_num = 0;
  std::vector<bool> tracks_used(no_track + no_object, false);
  std::vector<bool> objects_used(no_object, false);
  for (size_t i = 0; i < tracks_idx.size(); ++i) {
    if (tracks_idx[i] < 0 || tracks_idx[i] >= no_track || objects_idx[i] < 0 ||
        objects_idx[i] >= no_object) {
      continue;
    }
    if (association_mat(tracks_idx[i], objects_idx[i]) <
        assign_distance_maximum) {
      (*assignments)[assignments_num++] =
          std::make_pair(tracks_idx[i], objects_idx[i]);
      tracks_used[tracks_idx[i]] = true;
      objects_used[objects_idx[i]] = true;
    }
  }
  assignments->resize(assignments_num);
  unassigned_tracks->resize(association_mat.rows());
  int unassigned_tracks_num = 0;
  for (int i = 0; i < association_mat.rows(); ++i) {
    if (tracks_used[i] == false) {
      (*unassigned_tracks)[unassigned_tracks_num++] = i;
    }
  }
  unassigned_tracks->resize(unassigned_tracks_num);
  unassigned_objects->resize(association_mat.cols());
  int unassigned_objects_num = 0;
  for (int i = 0; i < association_mat.cols(); ++i) {
    if (objects_used[i] == false) {
      (*unassigned_objects)[unassigned_objects_num++] = i;
    }
  }
  unassigned_objects->resize(unassigned_objects_num);
}

}  // namespace perception
}  // namespace apollo
