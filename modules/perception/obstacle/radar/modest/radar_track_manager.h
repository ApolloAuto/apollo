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

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_RADAR_TRACK_MANAGER_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_RADAR_TRACK_MANAGER_H_

#include <mutex>
#include <utility>
#include <vector>

#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/radar/modest/radar_define.h"
#include "modules/perception/obstacle/radar/modest/radar_track.h"

namespace apollo {
namespace perception {

class RadarTrackManager {
 public:
  RadarTrackManager() {}
  ~RadarTrackManager() {}

  // @brief: process radar obstacles
  // @param [in]: built radar obstacles
  // @return nothing
  void Process(const SensorObjects &radar_obs);

  // @brief: update tracking state
  // @param [in]: built radar obstacles
  // @return nothing
  void Update(SensorObjects *radar_obs);

  // @brief match observation obstacles to existed tracking states by
  //            tracking id
  // @param [out]: assigement index pairs of observations and tracking states
  // @param [out]: indexs of unassigend tracking state
  // @param [out]: indexs of unassigned observation obstacles
  // @return nothing
  void AssignTrackObsIdMatch(const SensorObjects &radar_obs,
                             std::vector<std::pair<int, int>> *assignment,
                             std::vector<int> *unassigned_track,
                             std::vector<int> *unassigned_obs);

  // @brief update tracking state with assigned observation obstacle
  // @param [IN]: assigement index pairs of observations and tracking states
  // @return nothing
  void UpdateAssignedTrack(const SensorObjects &radar_obs,
                           const std::vector<std::pair<int, int>> &assignment);

  // @brief update tracking state of unassigned tracking state
  // @param [IN]: indexs of unassigend tracking state
  // @return nothing
  void UpdateUnassignedTrack(const double &timestamp,
                             const std::vector<int> &unassigned_track);

  // @brief delete stale tracking states
  // @return nothing
  void DeleteLostTrack();

  // @brief create a new tracking state with observation obstacles
  // @param [IN]: indexs of unassigend observation obstacles
  // @return nothing
  void CreateNewTrack(const SensorObjects &radar_obs,
                      const std::vector<int> &unassigned_obs);

  // @brief get radar obstacles
  // @return radar obstacles
  SensorObjects &GetRadarObs() { return radar_obs_; }

  // @brief get tracking state
  // @return tracking states
  std::vector<RadarTrack> &GetTracks() { return obs_tracks_; }

 private:
  double DistanceBetweenObs(const Object &obs1, double timestamp1,
                            const Object &obs2, double timestamp2);
  SensorObjects radar_obs_;
  std::vector<RadarTrack> obs_tracks_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_RADAR_TRACK_MANAGER_H_
