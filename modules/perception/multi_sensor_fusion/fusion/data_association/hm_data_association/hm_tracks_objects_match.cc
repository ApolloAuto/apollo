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
#include "modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association/hm_tracks_objects_match.h"

#include <map>
#include <numeric>
#include <utility>

#include "modules/perception/common/algorithm/graph/secure_matrix.h"

namespace apollo {
namespace perception {
namespace fusion {

double HMTrackersObjectsAssociation::s_match_distance_thresh_ = 4.0;
double HMTrackersObjectsAssociation::s_match_distance_bound_ = 100.0;
/* this is a slack threshold for camera 2 lidar/radar association.
 * consider the ave 2d-to-3d error is 7%, 30m is 15% of 200m, which
 * is 2 times of ave error around 200m. */
double HMTrackersObjectsAssociation::s_association_center_dist_threshold_ =
    30.0;

template <typename T>
void extract_vector(const std::vector<T>& vec,
                    const std::vector<size_t>& subset_inds,
                    std::vector<T>* sub_vec) {
  sub_vec->reserve(subset_inds.size());
  sub_vec->clear();
  for (auto subset_ind : subset_inds) {
    sub_vec->push_back(vec[subset_ind]);
  }
}

bool HMTrackersObjectsAssociation::Associate(
    const AssociationOptions& options, SensorFramePtr sensor_measurements,
    ScenePtr scene, AssociationResult* association_result) {
  const std::vector<SensorObjectPtr>& sensor_objects =
      sensor_measurements->GetForegroundObjects();
  const std::vector<TrackPtr>& fusion_tracks = scene->GetForegroundTracks();
  std::vector<std::vector<double>> association_mat;

  if (fusion_tracks.empty() || sensor_objects.empty()) {
    association_result->unassigned_tracks.resize(fusion_tracks.size());
    association_result->unassigned_measurements.resize(sensor_objects.size());
    std::iota(association_result->unassigned_tracks.begin(),
              association_result->unassigned_tracks.end(), 0);
    std::iota(association_result->unassigned_measurements.begin(),
              association_result->unassigned_measurements.end(), 0);
    return true;
  }

  std::string measurement_sensor_id = sensor_objects[0]->GetSensorId();
  double measurement_timestamp = sensor_objects[0]->GetTimestamp();
  track_object_distance_.ResetProjectionCache(measurement_sensor_id,
                                              measurement_timestamp);
  bool do_nothing = (measurement_sensor_id == "radar_front" ||
                     measurement_sensor_id == "radar_rear");
  IdAssign(fusion_tracks, sensor_objects, &association_result->assignments,
           &association_result->unassigned_tracks,
           &association_result->unassigned_measurements, do_nothing, false);

  Eigen::Affine3d pose;
  sensor_measurements->GetPose(&pose);
  Eigen::Vector3d ref_point = pose.translation();

  ADEBUG << "association_measurement_timestamp@" << measurement_timestamp;
  ComputeAssociationDistanceMat(fusion_tracks, sensor_objects, ref_point,
                                association_result->unassigned_tracks,
                                association_result->unassigned_measurements,
                                &association_mat);

  int num_track = static_cast<int>(fusion_tracks.size());
  int num_measurement = static_cast<int>(sensor_objects.size());
  association_result->track2measurements_dist.assign(num_track, 0);
  association_result->measurement2track_dist.assign(num_measurement, 0);
  std::vector<int> track_ind_g2l;
  track_ind_g2l.resize(num_track, -1);
  for (size_t i = 0; i < association_result->unassigned_tracks.size(); ++i) {
    track_ind_g2l[association_result->unassigned_tracks[i]] =
        static_cast<int>(i);
  }
  std::vector<int> measurement_ind_g2l;
  measurement_ind_g2l.resize(num_measurement, -1);
  std::vector<size_t> measurement_ind_l2g =
      association_result->unassigned_measurements;
  for (size_t i = 0; i < association_result->unassigned_measurements.size();
       ++i) {
    measurement_ind_g2l[association_result->unassigned_measurements[i]] =
        static_cast<int>(i);
  }
  std::vector<size_t> track_ind_l2g = association_result->unassigned_tracks;

  if (association_result->unassigned_tracks.empty() ||
      association_result->unassigned_measurements.empty()) {
    return true;
  }

  bool state = MinimizeAssignment(
      association_mat, track_ind_l2g, measurement_ind_l2g,
      &association_result->assignments, &association_result->unassigned_tracks,
      &association_result->unassigned_measurements);

  // start do post assign
  std::vector<TrackMeasurmentPair> post_assignments;
  PostIdAssign(fusion_tracks, sensor_objects,
               association_result->unassigned_tracks,
               association_result->unassigned_measurements, &post_assignments);
  association_result->assignments.insert(association_result->assignments.end(),
                                         post_assignments.begin(),
                                         post_assignments.end());

  GenerateUnassignedData(fusion_tracks.size(), sensor_objects.size(),
                         association_result->assignments,
                         &association_result->unassigned_tracks,
                         &association_result->unassigned_measurements);

  ComputeDistance(fusion_tracks, sensor_objects,
                  association_result->unassigned_tracks, track_ind_g2l,
                  measurement_ind_g2l, measurement_ind_l2g, association_mat,
                  association_result);

  AINFO << "association: measurement_num = " << sensor_objects.size()
        << ", track_num = " << fusion_tracks.size()
        << ", assignments = " << association_result->assignments.size()
        << ", unassigned_tracks = "
        << association_result->unassigned_tracks.size()
        << ", unassigned_measuremnets = "
        << association_result->unassigned_measurements.size();

  return state;
}

void HMTrackersObjectsAssociation::PostIdAssign(
    const std::vector<TrackPtr>& fusion_tracks,
    const std::vector<SensorObjectPtr>& sensor_objects,
    const std::vector<size_t>& unassigned_fusion_tracks,
    const std::vector<size_t>& unassigned_sensor_objects,
    std::vector<TrackMeasurmentPair>* post_assignments) {
  std::vector<size_t> valid_unassigned_tracks;
  valid_unassigned_tracks.reserve(unassigned_fusion_tracks.size());
  // only camera track
  auto is_valid_track = [](const TrackPtr& fusion_track) {
    SensorObjectConstPtr camera_obj = fusion_track->GetLatestCameraObject();
    return camera_obj != nullptr &&
           fusion_track->GetLatestLidarObject() == nullptr &&
           fusion_track->GetLatestRadarObject() == nullptr;
  };
  for (auto unassigned_track_id : unassigned_fusion_tracks) {
    if (is_valid_track(fusion_tracks[unassigned_track_id])) {
      valid_unassigned_tracks.push_back(unassigned_track_id);
    }
  }
  std::vector<TrackPtr> sub_tracks;
  std::vector<SensorObjectPtr> sub_objects;
  extract_vector(fusion_tracks, valid_unassigned_tracks, &sub_tracks);
  extract_vector(sensor_objects, unassigned_sensor_objects, &sub_objects);
  std::vector<size_t> tmp1, tmp2;
  IdAssign(sub_tracks, sub_objects, post_assignments, &tmp1, &tmp2, false,
           true);
  for (auto& post_assignment : *post_assignments) {
    post_assignment.first = valid_unassigned_tracks[post_assignment.first];
    post_assignment.second = unassigned_sensor_objects[post_assignment.second];
  }
}

bool HMTrackersObjectsAssociation::MinimizeAssignment(
    const std::vector<std::vector<double>>& association_mat,
    const std::vector<size_t>& track_ind_l2g,
    const std::vector<size_t>& measurement_ind_l2g,
    std::vector<TrackMeasurmentPair>* assignments,
    std::vector<size_t>* unassigned_tracks,
    std::vector<size_t>* unassigned_measurements) {
  algorithm::GatedHungarianMatcher<float>::OptimizeFlag opt_flag =
      algorithm::GatedHungarianMatcher<float>::OptimizeFlag::OPTMIN;
  algorithm::SecureMat<float>* global_costs = optimizer_.mutable_global_costs();
  int rows = static_cast<int>(unassigned_tracks->size());
  int cols = static_cast<int>(unassigned_measurements->size());

  global_costs->Resize(rows, cols);
  for (int r_i = 0; r_i < rows; r_i++) {
    for (int c_i = 0; c_i < cols; c_i++) {
      (*global_costs)(r_i, c_i) = static_cast<float>(association_mat[r_i][c_i]);
    }
  }
  std::vector<TrackMeasurmentPair> local_assignments;
  std::vector<size_t> local_unassigned_tracks;
  std::vector<size_t> local_unassigned_measurements;
  optimizer_.Match(static_cast<float>(s_match_distance_thresh_),
                   static_cast<float>(s_match_distance_bound_), opt_flag,
                   &local_assignments, &local_unassigned_tracks,
                   &local_unassigned_measurements);
  for (auto assign : local_assignments) {
    assignments->push_back(std::make_pair(track_ind_l2g[assign.first],
                                          measurement_ind_l2g[assign.second]));
  }
  unassigned_tracks->clear();
  unassigned_measurements->clear();
  for (auto un_track : local_unassigned_tracks) {
    unassigned_tracks->push_back(track_ind_l2g[un_track]);
  }
  for (auto un_mea : local_unassigned_measurements) {
    unassigned_measurements->push_back(measurement_ind_l2g[un_mea]);
  }
  return true;
}

void HMTrackersObjectsAssociation::ComputeDistance(
    const std::vector<TrackPtr>& fusion_tracks,
    const std::vector<SensorObjectPtr>& sensor_objects,
    const std::vector<size_t>& unassigned_fusion_tracks,
    const std::vector<int>& track_ind_g2l,
    const std::vector<int>& measurement_ind_g2l,
    const std::vector<size_t>& measurement_ind_l2g,
    const std::vector<std::vector<double>>& association_mat,
    AssociationResult* association_result) {
  for (size_t i = 0; i < association_result->assignments.size(); ++i) {
    int track_ind = static_cast<int>(association_result->assignments[i].first);
    int measurement_ind =
        static_cast<int>(association_result->assignments[i].second);
    int track_ind_loc = track_ind_g2l[track_ind];
    int measurement_ind_loc = measurement_ind_g2l[measurement_ind];
    if (track_ind_loc >= 0 && measurement_ind_loc >= 0) {
      association_result->track2measurements_dist[track_ind] =
          association_mat[track_ind_loc][measurement_ind_loc];
      association_result->measurement2track_dist[measurement_ind] =
          association_mat[track_ind_loc][measurement_ind_loc];
    }
  }
  for (size_t i = 0; i < association_result->unassigned_tracks.size(); ++i) {
    int track_ind = static_cast<int>(unassigned_fusion_tracks[i]);
    int track_ind_loc = track_ind_g2l[track_ind];
    association_result->track2measurements_dist[track_ind] =
        association_mat[track_ind_loc][0];
    int min_m_loc = 0;
    for (size_t j = 1; j < association_mat[track_ind_loc].size(); j++) {
      if (association_result->track2measurements_dist[track_ind] >
          association_mat[track_ind_loc][j]) {
        association_result->track2measurements_dist[track_ind] =
            association_mat[track_ind_loc][j];
        min_m_loc = static_cast<int>(j);
      }
    }
    int min_m_ind = static_cast<int>(measurement_ind_l2g[min_m_loc]);
    const SensorObjectPtr& min_sensor_object = sensor_objects[min_m_ind];
    const TrackPtr& fusion_track = fusion_tracks[track_ind];
    SensorObjectConstPtr lidar_object = fusion_track->GetLatestLidarObject();
    SensorObjectConstPtr radar_object = fusion_track->GetLatestRadarObject();
    if (IsCamera(min_sensor_object)) {
      // TODO(linjian) not reasonable,
      // just for return dist score, the dist score is
      // a similarity probability [0, 1] 1 is the best
      association_result->track2measurements_dist[track_ind] = 0.0;
      for (size_t j = 0; j < association_mat[track_ind_loc].size(); ++j) {
        double dist_score = 0.0;
        if (lidar_object != nullptr) {
          dist_score = track_object_distance_.ComputeLidarCameraSimilarity(
              lidar_object, sensor_objects[measurement_ind_l2g[j]],
              IsLidar(sensor_objects[measurement_ind_l2g[j]]));
        } else if (radar_object != nullptr) {
          dist_score = track_object_distance_.ComputeRadarCameraSimilarity(
              radar_object, sensor_objects[measurement_ind_l2g[j]]);
        }
        association_result->track2measurements_dist[track_ind] = std::max(
            association_result->track2measurements_dist[track_ind], dist_score);
      }
    }
  }
  for (size_t i = 0; i < association_result->unassigned_measurements.size();
       ++i) {
    int m_ind =
        static_cast<int>(association_result->unassigned_measurements[i]);
    int m_ind_loc = measurement_ind_g2l[m_ind];
    association_result->measurement2track_dist[m_ind] =
        association_mat[0][m_ind_loc];
    for (size_t j = 1; j < association_mat.size(); j++) {
      if (association_result->measurement2track_dist[m_ind] >
          association_mat[j][m_ind_loc]) {
        association_result->measurement2track_dist[m_ind] =
            association_mat[j][m_ind_loc];
      }
    }
  }
}

void HMTrackersObjectsAssociation::ComputeAssociationDistanceMat(
    const std::vector<TrackPtr>& fusion_tracks,
    const std::vector<SensorObjectPtr>& sensor_objects,
    const Eigen::Vector3d& ref_point,
    const std::vector<size_t>& unassigned_tracks,
    const std::vector<size_t>& unassigned_measurements,
    std::vector<std::vector<double>>* association_mat) {
  TrackObjectDistanceOptions opt;
  Eigen::Vector3d tmp = Eigen::Vector3d::Zero();
  opt.ref_point = &tmp;
  association_mat->resize(unassigned_tracks.size());
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    size_t fusion_idx = unassigned_tracks[i];
    (*association_mat)[i].resize(unassigned_measurements.size());
    const TrackPtr& fusion_track = fusion_tracks[fusion_idx];
    for (size_t j = 0; j < unassigned_measurements.size(); ++j) {
      size_t sensor_idx = unassigned_measurements[j];
      const SensorObjectPtr& sensor_object = sensor_objects[sensor_idx];
      double distance = s_match_distance_thresh_;
      double center_dist =
          (sensor_object->GetBaseObject()->center -
           fusion_track->GetFusedObject()->GetBaseObject()->center).norm();
      if (center_dist < s_association_center_dist_threshold_) {
        distance =
            track_object_distance_.Compute(fusion_track, sensor_object, opt);
      } else {
        ADEBUG << "center_distance " << center_dist
               << " exceeds slack threshold "
               << s_association_center_dist_threshold_
               << ", track_id: " << fusion_track->GetTrackId()
               << ", obs_id: " << sensor_object->GetBaseObject()->track_id;
      }
      (*association_mat)[i][j] = distance;
      ADEBUG << "track_id: " << fusion_track->GetTrackId()
             << ", obs_id: " << sensor_object->GetBaseObject()->track_id
             << ", distance: " << distance;
    }
  }
}

void HMTrackersObjectsAssociation::IdAssign(
    const std::vector<TrackPtr>& fusion_tracks,
    const std::vector<SensorObjectPtr>& sensor_objects,
    std::vector<TrackMeasurmentPair>* assignments,
    std::vector<size_t>* unassigned_fusion_tracks,
    std::vector<size_t>* unassigned_sensor_objects, bool do_nothing,
    bool post) {
  size_t num_track = fusion_tracks.size();
  size_t num_obj = sensor_objects.size();
  if (num_track == 0 || num_obj == 0 || do_nothing) {
    unassigned_fusion_tracks->resize(num_track);
    unassigned_sensor_objects->resize(num_obj);
    std::iota(unassigned_fusion_tracks->begin(),
              unassigned_fusion_tracks->end(), 0);
    std::iota(unassigned_sensor_objects->begin(),
              unassigned_sensor_objects->end(), 0);
    return;
  }

  const std::string sensor_id = sensor_objects[0]->GetSensorId();

  std::map<int, int> sensor_id_2_track_ind;
  for (size_t i = 0; i < num_track; ++i) {
    SensorObjectConstPtr obj = fusion_tracks[i]->GetSensorObject(sensor_id);
    /* when camera system has sub-fusion of obstacle & narrow, they share
     * the same track-id sequence. thus, latest camera object is ok for
     * camera id assign and its information is more up to date. */
    if (sensor_id == "front_6mm" || sensor_id == "front_12mm" ||
        sensor_id == "camera_rear" || sensor_id == "camera_front") {
      obj = fusion_tracks[i]->GetLatestCameraObject();
    }
    if (obj == nullptr) {
      continue;
    }
    sensor_id_2_track_ind[obj->GetBaseObject()->track_id] = static_cast<int>(i);
  }
  std::vector<bool> fusion_used(num_track, false);
  std::vector<bool> sensor_used(num_obj, false);
  for (size_t i = 0; i < num_obj; ++i) {
    int track_id = sensor_objects[i]->GetBaseObject()->track_id;
    auto it = sensor_id_2_track_ind.find(track_id);

    // In id_assign, we don't assign the narrow camera object
    // with the track which only have narrow camera object
    // In post id_assign, we do this.
    if (!post && (sensor_id == "front_6mm" || sensor_id == "front_12mm" ||
                  sensor_id == "camera_rear" || sensor_id == "camera_front")) {
      continue;
    }

    if (it != sensor_id_2_track_ind.end()) {
      sensor_used[i] = true;
      fusion_used[it->second] = true;
      assignments->push_back(std::make_pair(it->second, i));
    }
  }
  for (size_t i = 0; i < fusion_used.size(); ++i) {
    if (!fusion_used[i]) {
      unassigned_fusion_tracks->push_back(i);
    }
  }
  for (size_t i = 0; i < sensor_used.size(); ++i) {
    if (!sensor_used[i]) {
      unassigned_sensor_objects->push_back(i);
    }
  }
}

void HMTrackersObjectsAssociation::GenerateUnassignedData(
    size_t track_num, size_t objects_num,
    const std::vector<TrackMeasurmentPair>& assignments,
    std::vector<size_t>* unassigned_tracks,
    std::vector<size_t>* unassigned_objects) {
  std::vector<bool> track_flags(track_num, false);
  std::vector<bool> objects_flags(objects_num, false);
  for (auto assignment : assignments) {
    track_flags[assignment.first] = true;
    objects_flags[assignment.second] = true;
  }
  unassigned_tracks->clear(), unassigned_tracks->reserve(track_num);
  unassigned_objects->clear(), unassigned_objects->reserve(objects_num);
  for (size_t i = 0; i < track_num; ++i) {
    if (!track_flags[i]) {
      unassigned_tracks->push_back(i);
    }
  }
  for (size_t i = 0; i < objects_num; ++i) {
    if (!objects_flags[i]) {
      unassigned_objects->push_back(i);
    }
  }
}

PERCEPTION_REGISTER_ASSOCIATION(HMTrackersObjectsAssociation)

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
