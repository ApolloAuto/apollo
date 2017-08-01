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

#include <utility>
#include <vector>
#include "modules/common/log.h"
#include "modules/perception/obstacle/common/graph_util.h"
#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/common/hungarian_bigraph_matcher.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/hungarian_matcher.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/track_object_distance.h"

namespace apollo {
namespace perception {

float HungarianMatcher::s_max_match_distance_ = 4.0f;

HungarianMatcher::HungarianMatcher() {
}

HungarianMatcher::~HungarianMatcher() {
}

bool HungarianMatcher::SetMaxMatchDistance(
  float max_match_distance) {
  if (max_match_distance >= 0) {
    s_max_match_distance_ = max_match_distance;
    return true;
  } else {
    AERROR << "invalid max_match_distance for HungraianMatcher.";
    return false;
  }
}

void HungarianMatcher::Match(std::vector<TrackedObjectPtr>* objects,
  const std::vector<ObjectTrackPtr>& tracks,
  const std::vector<Eigen::VectorXf>& tracks_predict,
  const double time_diff,
  std::vector<TrackObjectPair>* assignments,
  std::vector<int>* unassigned_tracks,
  std::vector<int>* unassigned_objects) {
  // Match detected objects to tracks
  float max_dist = s_max_match_distance_;

  // 1. computing association matrix
  Eigen::MatrixXf association_mat(tracks.size(), objects->size());
  ComputeAssociateMatrix(tracks, tracks_predict, (*objects), time_diff,
    &association_mat);

  // 2. computing connected components
  std::vector<std::vector<int> > obj_components;
  std::vector<std::vector<int> > track_components;
  ComputeConnectedComponents(association_mat, max_dist, &track_components,
    &obj_components);

  AINFO << "HungarianMatcher: partition graph into "
    << track_components.size() << " sub-graphs.";

  // 3. matching each sub-graph
  std::vector<int> track_l2g;
  std::vector<int> obj_l2g;
  std::vector<int> unassigned_tracks_loc;
  std::vector<int> unassigned_objects_loc;
  std::vector<TrackObjectPair> assignments_loc;
  assignments->clear();
  unassigned_tracks->clear();
  unassigned_objects->clear();

  for (int i = 0; i < track_components.size(); i++) {
    if (track_components[i].empty() || obj_components[i].empty()) {
      if (track_components[i].empty()) {
        for (int j = 0; j < obj_components[i].size(); j++) {
          unassigned_objects->push_back(obj_components[i][j]);
        }
      } else {
        for (int j = 0; j < track_components[i].size(); j++) {
          unassigned_tracks->push_back(track_components[i][j]);
        }
      }
      continue;
    } else if (track_components[i].size() == 1 &&
      obj_components[i].size() == 1) {
      if (association_mat(track_components[i][0], obj_components[i][0])
        <= max_dist) {
        assignments->push_back(std::make_pair(track_components[i][0],
          obj_components[i][0]));
        // update association score for assigned objects
        float association_score = association_mat(track_components[i][0],
          obj_components[i][0]);
        int obj_id = obj_components[i][0];
        (*objects)[obj_id]->association_score = association_score;
      } else {
        unassigned_objects->push_back(obj_components[i][0]);
        unassigned_tracks->push_back(track_components[i][0]);
      }
      continue;
    }
    Eigen::MatrixXf mat_loc(track_components[i].size(),
      obj_components[i].size());
    track_l2g.resize(track_components[i].size());
    obj_l2g.resize(obj_components[i].size());
    for (int j = 0; j < track_components[i].size(); j++) {
      track_l2g[j] = track_components[i][j];
      for (int k = 0; k < obj_components[i].size(); k++) {
        if (j == 0) {
          obj_l2g[k] = obj_components[i][k];
        }
        mat_loc(j, k) = association_mat(track_components[i][j],
          obj_components[i][k]);
      }
    }
    assignments_loc.resize(mat_loc.cols());
    unassigned_tracks_loc.assign(mat_loc.rows(), -1);
    unassigned_objects_loc.assign(mat_loc.cols(), -1);

    AssignObjectsToTracksUsingNullTracks(mat_loc, max_dist, &assignments_loc,
      &unassigned_tracks_loc, &unassigned_objects_loc);
    for (int j = 0; j < assignments_loc.size(); j++) {
      int track_gid = track_l2g[assignments_loc[j].first];
      int obj_gid = obj_l2g[assignments_loc[j].second];
      assignments->push_back(std::make_pair(track_gid, obj_gid));
      // update association score for assigned objects
      float association_score = association_mat(track_gid, obj_gid);
      (*objects)[obj_gid]->association_score = association_score;
    }
    for (int j = 0; j < unassigned_tracks_loc.size(); j++) {
      int track_gid = track_l2g[unassigned_tracks_loc[j]];
      unassigned_tracks->push_back(track_gid);
    }
    for (int j = 0; j < unassigned_objects_loc.size(); j++) {
      int obj_gid = obj_l2g[unassigned_objects_loc[j]];
      unassigned_objects->push_back(obj_gid);
    }
  }
}

void HungarianMatcher::ComputeAssociateMatrix(
  const std::vector<ObjectTrackPtr>& tracks,
  const std::vector<Eigen::VectorXf>& tracks_predict,
  const std::vector<TrackedObjectPtr>& new_objects,
  const double time_diff,
  Eigen::MatrixXf* association_mat) {
  // Compute matrix of association distance
  for (size_t i = 0; i < tracks.size(); ++i) {
    for (size_t j = 0; j < new_objects.size(); ++j) {
      (*association_mat)(i, j) = ComputeTrackObjectDistance(tracks[i],
        tracks_predict[i], new_objects[j], time_diff);
    }
  }
}

float HungarianMatcher::ComputeTrackObjectDistance(
  const ObjectTrackPtr& track,
  const Eigen::VectorXf& track_predict,
  const TrackedObjectPtr& new_object,
  const double time_diff) const {
  // Compute distance of given track & object
  return TrackObjectDistance::ComputeDistance(track, track_predict,
    new_object, time_diff);
}

void HungarianMatcher::ComputeConnectedComponents(
  const Eigen::MatrixXf& association_mat,
  const float connected_threshold,
  std::vector<std::vector<int> >* track_components,
  std::vector<std::vector<int> >* obj_components) {
  // Compute connected components within given threshold
  int no_track = association_mat.rows();
  int no_obj = association_mat.cols();
  std::vector<std::vector<int> > nb_graph;
  nb_graph.resize(no_track + no_obj);
  for (int i = 0; i < no_track; i++) {
    for (int j = 0; j < no_obj; j++) {
      if (association_mat(i, j) <= connected_threshold) {
        nb_graph[i].push_back(no_track + j);
        nb_graph[j + no_track].push_back(i);
      }
    }
  }

  std::vector<std::vector<int> > components;
  connected_component_analysis(nb_graph, components);
  track_components->clear();
  track_components->resize(components.size());
  obj_components->clear();
  obj_components->resize(components.size());
  for (size_t i = 0; i < components.size(); i++) {
    for (size_t j = 0; j < components[i].size(); j++) {
      int id = components[i][j];
      if (id < no_track) {
        (*track_components)[i].push_back(id);
      } else {
        id -= no_track;
        (*obj_components)[i].push_back(id);
      }
    }
  }
}

void HungarianMatcher::AssignObjectsToTracks(
  const Eigen::MatrixXf& association_mat,
  const double max_distance,
  std::vector<TrackObjectPair>* assignments,
  std::vector<int>* unassigned_tracks,
  std::vector<int>* unassigned_objects) {
  // Assign objects to tracks without null tracks setup
  float max_dist = max_distance;
  std::vector<int> tracks_idx;
  std::vector<int> objects_idx;
  std::vector< std::vector<double> > cost(association_mat.rows());
  for (int i = 0; i < association_mat.rows(); i++) {
    cost[i].resize(association_mat.cols());
    for (int j = 0; j < association_mat.cols(); j++) {
      cost[i][j] = association_mat(i, j);
    }
  }

  HungarianOptimizer hungarian_optimizer(cost);
  hungarian_optimizer.minimize(&tracks_idx, &objects_idx);

  int assignments_num = 0;
  std::vector<bool> tracks_used(association_mat.rows(), false);
  std::vector<bool> objects_used(association_mat.cols(), false);
  for (size_t i = 0; i < tracks_idx.size(); ++i) {
    if (tracks_idx[i] < 0 || tracks_idx[i] >= association_mat.rows() ||
      objects_idx[i] < 0 || objects_idx[i] >= association_mat.cols()) {
      continue;
    }
    if (association_mat(tracks_idx[i], objects_idx[i]) <= max_dist) {
      (*assignments)[assignments_num++] = std::make_pair(tracks_idx[i],
        objects_idx[i]);
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

void HungarianMatcher::AssignObjectsToTracksUsingNullTracks(
  const Eigen::MatrixXf& association_mat,
  const double max_distance,
  std::vector<TrackObjectPair>* assignments,
  std::vector<int>* unassigned_tracks,
  std::vector<int>* unassigned_objects) {
  // Assign objects to tracks with null tracks setup
  float max_dist = max_distance;
  std::vector<int> tracks_idx;
  std::vector<int> objects_idx;
  int no_track = association_mat.rows();
  int no_obj = association_mat.cols();

  std::vector< std::vector<double> > cost(no_track + no_obj);

  for (int i = 0; i < no_track; i++) {
    cost[i].resize(association_mat.cols());
    for (int j = 0; j < association_mat.cols(); j++) {
      cost[i][j] = association_mat(i, j);
    }
  }
  for (int i = 0; i < no_obj; i++) {
    cost[i + no_track].resize(no_obj);
    for (int j = 0; j < no_obj; j++) {
      if (j == i) {
        cost[i + no_track][j] = max_dist * 1.2f;
      } else {
        cost[i + no_track][j] = 999999.0f;
      }
    }
  }

  HungarianOptimizer hungarian_optimizer(cost);
  hungarian_optimizer.minimize(&tracks_idx, &objects_idx);

  int assignments_num = 0;
  std::vector<bool> tracks_used(no_track + no_obj, false);
  std::vector<bool> objects_used(no_obj, false);
  for (size_t i = 0; i < tracks_idx.size(); ++i) {
    if (tracks_idx[i] < 0 || tracks_idx[i] >= no_track ||
      objects_idx[i] < 0 || objects_idx[i] >= no_obj) {
      continue;
    }
    if (association_mat(tracks_idx[i], objects_idx[i]) < max_dist) {
      (*assignments)[assignments_num++] = std::make_pair(tracks_idx[i],
        objects_idx[i]);
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
