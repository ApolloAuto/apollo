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

#include "modules/perception/obstacle/radar/modest/radar_track_manager.h"

#include <memory>
#include <utility>

namespace apollo {
namespace perception {

void RadarTrackManager::Process(const SensorObjects &radar_obs) {
  radar_obs_ = radar_obs;
  Update(&radar_obs_);
}

void RadarTrackManager::Update(SensorObjects *radar_obs) {
  std::vector<std::pair<int, int>> assignment;
  std::vector<int> unassigned_track;
  std::vector<int> unassigned_obs;
  AssignTrackObsIdMatch(*radar_obs, &assignment, &unassigned_track,
                        &unassigned_obs);
  UpdateAssignedTrack(*radar_obs, assignment);
  UpdateUnassignedTrack((*radar_obs).timestamp, unassigned_track);
  DeleteLostTrack();
  CreateNewTrack(*radar_obs, unassigned_obs);
}

void RadarTrackManager::AssignTrackObsIdMatch(
    const SensorObjects &radar_obs,
    std::vector<std::pair<int, int>> *assignment,
    std::vector<int> *unassigned_track, std::vector<int> *unassigned_obs) {
  assignment->resize(obs_tracks_.size() * 2);
  int assignment_num = 0;
  std::vector<bool> track_used(obs_tracks_.size(), false);
  std::vector<bool> obs_used(radar_obs.objects.size(), false);
  for (size_t i = 0; i < obs_tracks_.size(); i++) {
    std::shared_ptr<Object> obs;
    obs = obs_tracks_[i].GetObsRadar();
    if (obs == nullptr) {
      continue;
    }
    double timestamp_track = obs_tracks_[i].GetTimestamp();
    double timestamp_obs = radar_obs.timestamp;
    for (size_t j = 0; j < radar_obs.objects.size(); j++) {
      double distance = DistanceBetweenObs(
          *obs, timestamp_track, *(radar_obs.objects[j]), timestamp_obs);
      if (obs->track_id == radar_obs.objects[j]->track_id &&
          distance < RADAR_TRACK_THRES) {
        assignment->at(assignment_num++) = std::make_pair(i, j);
        track_used[i] = true;
        obs_used[j] = true;
        obs_tracks_[i].IncreaseTrackedTimes();
      }
    }
  }

  assignment->resize(assignment_num);
  unassigned_track->resize(obs_tracks_.size());
  int unassigned_track_num = 0;
  for (size_t i = 0; i < track_used.size(); i++) {
    if (track_used[i] == false) {
      unassigned_track->at(unassigned_track_num++) = i;
    }
  }
  unassigned_track->resize(unassigned_track_num);

  unassigned_obs->resize(radar_obs.objects.size());
  int unassigned_obs_num = 0;
  for (size_t i = 0; i < obs_used.size(); i++) {
    if (obs_used[i] == false) {
      unassigned_obs->at(unassigned_obs_num++) = i;
    }
  }
  unassigned_obs->resize(unassigned_obs_num);
}

void RadarTrackManager::UpdateAssignedTrack(
    const SensorObjects &radar_obs,
    const std::vector<std::pair<int, int>> &assignment) {
  for (size_t i = 0; i < assignment.size(); i++) {
    obs_tracks_[assignment[i].first].UpdataObsRadar(
        radar_obs.objects[assignment[i].second], radar_obs.timestamp);
  }
}

void RadarTrackManager::UpdateUnassignedTrack(
    const double &timestamp, const std::vector<int> &unassigned_track) {
  double time_stamp = timestamp;
  for (size_t i = 0; i < unassigned_track.size(); i++) {
    if (obs_tracks_[unassigned_track[i]].GetObsRadar() != nullptr) {
      double radar_time = obs_tracks_[unassigned_track[i]].GetTimestamp();
      double time_diff = fabs(time_stamp - radar_time);
      if (time_diff > RADAR_TRACK_TIME_WIN) {
        obs_tracks_[unassigned_track[i]].SetObsRadar(nullptr);
      }
    }
  }
}

void RadarTrackManager::DeleteLostTrack() {
  int track_num = 0;
  for (size_t i = 0; i < obs_tracks_.size(); i++) {
    if (obs_tracks_[i].GetObsRadar() != nullptr) {
      obs_tracks_[track_num++] = obs_tracks_[i];
    }
  }
  obs_tracks_.resize(track_num);
}

void RadarTrackManager::CreateNewTrack(const SensorObjects &radar_obs,
                                       const std::vector<int> &unassigned_obs) {
  for (size_t i = 0; i < unassigned_obs.size(); i++) {
    obs_tracks_.push_back(RadarTrack(*(radar_obs.objects[unassigned_obs[i]]),
                                     radar_obs.timestamp));
  }
}

double RadarTrackManager::DistanceBetweenObs(const Object &obs1,
                                             double timestamp1,
                                             const Object &obs2,
                                             double timestamp2) {
  double time_diff = timestamp2 - timestamp1;
  return (obs2.center - obs1.center - obs1.velocity * time_diff).head(2).norm();
}

}  // namespace perception
}  // namespace apollo
