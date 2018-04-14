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

#include <iomanip>
#include <sstream>

#include "modules/common/log.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_hm_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track_object_distance.h"

namespace apollo {
namespace perception {

bool PbfHmTrackObjectMatcher::Match(
    const std::vector<PbfTrackPtr> &fusion_tracks,
    const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
    const TrackObjectMatcherOptions &options,
    std::vector<std::pair<int, int>> *assignments,
    std::vector<int> *unassigned_fusion_tracks,
    std::vector<int> *unassigned_sensor_objects,
    std::vector<double> *track2measurements_dist,
    std::vector<double> *measurement2track_dist) {
  CHECK_NOTNULL(assignments);
  CHECK_NOTNULL(unassigned_fusion_tracks);
  CHECK_NOTNULL(unassigned_sensor_objects);
  CHECK_NOTNULL(track2measurements_dist);
  CHECK_NOTNULL(measurement2track_dist);

  if (options.ref_point == nullptr) {
    AERROR << "reference points is nullptr!";
    return false;
  }

  IdAssign(fusion_tracks, sensor_objects, assignments, unassigned_fusion_tracks,
           unassigned_sensor_objects);
  ADEBUG << "Num of fusion tracks = " << fusion_tracks.size()
         << ", num of sensor objects = " << sensor_objects.size()
         << ", num of assignments = " << assignments->size();

  std::vector<std::vector<double>> association_mat;
  ComputeAssociationMat(fusion_tracks, sensor_objects,
                        *unassigned_fusion_tracks, *unassigned_sensor_objects,
                        *(options.ref_point), &association_mat);

  track2measurements_dist->assign(fusion_tracks.size(), 0);
  measurement2track_dist->assign(sensor_objects.size(), 0);

  std::vector<int> track_ind_g2l(fusion_tracks.size(), -1);
  for (size_t i = 0; i < unassigned_fusion_tracks->size(); ++i) {
    track_ind_g2l[unassigned_fusion_tracks->at(i)] = i;
  }

  std::vector<int> measurement_ind_g2l(sensor_objects.size(), -1);
  for (size_t i = 0; i < unassigned_sensor_objects->size(); ++i) {
    measurement_ind_g2l[unassigned_sensor_objects->at(i)] = i;
  }

  if (unassigned_fusion_tracks->empty() || unassigned_sensor_objects->empty()) {
    return true;
  }

  bool state = HmAssign(association_mat, assignments, unassigned_fusion_tracks,
                        unassigned_sensor_objects);

  for (const auto &track_measurement_pair : *assignments) {
    const int track_ind = track_measurement_pair.first;
    const int measurement_ind = track_measurement_pair.second;
    ADEBUG << "track_ind is matched to measurement_ind for sensor "
           << sensor_objects[0]->sensor_id << " " << track_ind << " "
           << measurement_ind;
    const int track_ind_loc = track_ind_g2l[track_ind];
    const int measurement_ind_loc = measurement_ind_g2l[measurement_ind];
    if (track_ind_loc >= 0 && measurement_ind_loc >= 0) {
      track2measurements_dist->at(track_ind) =
          association_mat[track_ind_loc][measurement_ind_loc];
      measurement2track_dist->at(measurement_ind) =
          association_mat[track_ind_loc][measurement_ind_loc];
    }
  }
  for (size_t i = 0; i < unassigned_fusion_tracks->size(); ++i) {
    const int track_ind = unassigned_fusion_tracks->at(i);
    const int track_ind_loc = track_ind_g2l[track_ind];
    track2measurements_dist->at(track_ind) = association_mat[track_ind_loc][0];
    for (size_t j = 1; j < association_mat[track_ind_loc].size(); ++j) {
      if (track2measurements_dist->at(track_ind) >
          association_mat[track_ind_loc][j]) {
        track2measurements_dist->at(track_ind) =
            association_mat[track_ind_loc][j];
      }
    }
  }

  for (const int m_ind : *unassigned_sensor_objects) {
    const int m_ind_loc = measurement_ind_g2l[m_ind];
    measurement2track_dist->at(m_ind) = association_mat[0][m_ind_loc];
    for (const auto &asso_mat_row : association_mat) {
      if (measurement2track_dist->at(m_ind) > asso_mat_row[m_ind_loc]) {
        measurement2track_dist->at(m_ind) = asso_mat_row[m_ind_loc];
      }
    }
  }
  return state;
}

std::string PbfHmTrackObjectMatcher::name() const {
  return "PbfHmTrackObjectMatcher";
}

void PbfHmTrackObjectMatcher::ComputeAssociationMat(
    const std::vector<PbfTrackPtr> &fusion_tracks,
    const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
    const std::vector<int> &unassigned_fusion_tracks,
    const std::vector<int> &unassigned_sensor_objects,
    const Eigen::Vector3d &ref_point,
    std::vector<std::vector<double>> *association_mat) {
  CHECK_NOTNULL(association_mat);

  PbfTrackObjectDistance pbf_distance;
  Eigen::Vector3d local_ref_point = ref_point;
  TrackObjectDistanceOptions options;
  options.ref_point = &local_ref_point;
  association_mat->resize(unassigned_fusion_tracks.size());
  for (size_t i = 0; i < unassigned_fusion_tracks.size(); ++i) {
    int fusion_idx = unassigned_fusion_tracks[i];
    (*association_mat)[i].resize(unassigned_sensor_objects.size());
    const PbfTrackPtr &fusion_track = fusion_tracks[fusion_idx];
    for (size_t j = 0; j < unassigned_sensor_objects.size(); ++j) {
      int sensor_idx = unassigned_sensor_objects[j];
      const std::shared_ptr<PbfSensorObject> &sensor_object =
          sensor_objects[sensor_idx];
      double distance =
          pbf_distance.Compute(fusion_track, sensor_object, options);
      ADEBUG << "sensor distance:" << distance;
      (*association_mat)[i][j] = distance;
    }
  }
}

bool PbfHmTrackObjectMatcher::HmAssign(
    const std::vector<std::vector<double>> &association_mat,
    std::vector<std::pair<int, int>> *assignments,
    std::vector<int> *unassigned_fusion_tracks,
    std::vector<int> *unassigned_sensor_objects) {
  double max_dist = s_max_match_distance_;
  std::vector<std::vector<int>> fusion_components;
  std::vector<std::vector<int>> sensor_components;
  ComputeConnectedComponents(association_mat, max_dist, &fusion_components,
                             &sensor_components);

  if (fusion_components.size() != sensor_components.size()) {
    AERROR << "fusion component size it not equal to sensor component size.";
    return false;
  }
  for (size_t i = 0; i < fusion_components.size(); ++i) {
    if (fusion_components[i].empty() || sensor_components[i].empty()) {
      continue;
    } else if (fusion_components[i].size() == 1 &&
               sensor_components[i].size() == 1) {
      int idx_f = fusion_components[i][0];
      int idx_s = sensor_components[i][0];
      if (association_mat[idx_f][idx_s] < max_dist) {
        auto assignment = std::make_pair(unassigned_fusion_tracks->at(idx_f),
                                         unassigned_sensor_objects->at(idx_s));
        assignments->push_back(assignment);
        unassigned_fusion_tracks->at(idx_f) = -1;
        unassigned_sensor_objects->at(idx_s) = -1;
      }
      continue;
    }

    std::vector<std::vector<double>> loc_mat;
    std::vector<int> fusion_l2g;
    std::vector<int> sensor_l2g;
    fusion_l2g.resize(fusion_components[i].size());
    sensor_l2g.resize(sensor_components[i].size());
    loc_mat.resize(fusion_components[i].size());
    for (size_t j = 0; j < fusion_components[i].size(); ++j) {
      loc_mat[j].resize(sensor_components[i].size());
      fusion_l2g[j] = fusion_components[i][j];
      for (size_t k = 0; k < sensor_components[i].size(); ++k) {
        if (j == 0) {
          sensor_l2g[k] = sensor_components[i][k];
        }
        loc_mat[j][k] =
            association_mat[fusion_components[i][j]][sensor_components[i][k]];
      }
    }

    std::vector<int> fusion_idxs;
    std::vector<int> sensor_idxs;
    if (loc_mat.size() != 0 && loc_mat[0].size() != 0) {
      MinimizeAssignment(loc_mat, &fusion_idxs, &sensor_idxs);
    }

    for (size_t j = 0; j < fusion_idxs.size(); ++j) {
      int f_idx = fusion_idxs[j];
      int s_idx = sensor_idxs[j];
      if (loc_mat[f_idx][s_idx] < max_dist) {
        int gf_idx = fusion_l2g[f_idx];
        int gs_idx = sensor_l2g[s_idx];
        auto assignment = std::make_pair((*unassigned_fusion_tracks)[gf_idx],
                                         (*unassigned_sensor_objects)[gs_idx]);
        assignments->push_back(assignment);
        (*unassigned_fusion_tracks)[gf_idx] = -1;
        (*unassigned_sensor_objects)[gs_idx] = -1;
      }
    }
  }

  int unassigned_fusion_num = 0;
  for (size_t i = 0; i < unassigned_fusion_tracks->size(); ++i) {
    if ((*unassigned_fusion_tracks)[i] >= 0) {
      (*unassigned_fusion_tracks)[unassigned_fusion_num++] =
          (*unassigned_fusion_tracks)[i];
    }
  }
  (*unassigned_fusion_tracks).resize(unassigned_fusion_num);

  int unassigned_sensor_num = 0;
  for (size_t i = 0; i < unassigned_sensor_objects->size(); ++i) {
    if ((*unassigned_sensor_objects)[i] >= 0) {
      (*unassigned_sensor_objects)[unassigned_sensor_num++] =
          (*unassigned_sensor_objects)[i];
    }
  }
  unassigned_sensor_objects->resize(unassigned_sensor_num);
  return true;
}

void PbfHmTrackObjectMatcher::MinimizeAssignment(
    const std::vector<std::vector<double>> &association_mat,
    std::vector<int> *ref_idx, std::vector<int> *new_idx) {
  std::vector<std::vector<double>> cost(association_mat.size());
  for (size_t i = 0; i < association_mat.size(); ++i) {
    cost[i].resize(association_mat[i].size());
    for (size_t j = 0; j < association_mat[0].size(); ++j) {
      cost[i][j] = association_mat[i][j];
    }
  }

  HungarianOptimizer hungarian_optimizer(cost);
  hungarian_optimizer.minimize(ref_idx, new_idx);
}

bool PbfHmTrackObjectMatcher::Init() { return true; }

void PbfHmTrackObjectMatcher::ComputeConnectedComponents(
    const std::vector<std::vector<double>> &association_mat,
    const float connected_threshold,
    std::vector<std::vector<int>> *track_components,
    std::vector<std::vector<int>> *obj_components) {
  int no_track = association_mat.size();
  int no_obj = 0;
  if (no_track != 0) {
    no_obj = association_mat[0].size();
  }

  std::vector<std::vector<int>> nb_graph;
  nb_graph.resize(no_track + no_obj);
  for (int i = 0; i < no_track; i++) {
    for (int j = 0; j < no_obj; j++) {
      if (association_mat[i][j] < connected_threshold) {
        nb_graph[i].push_back(no_track + j);
        nb_graph[j + no_track].push_back(i);
      }
    }
  }

  std::vector<std::vector<int>> components;
  ConnectedComponentAnalysis(nb_graph, &components);
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

}  // namespace perception
}  // namespace apollo
