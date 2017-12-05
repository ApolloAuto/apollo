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

  void Process(const SensorObjects &radar_obs);

  // update tracking state using kalman filter
  void Update(SensorObjects* radar_obs);

  // match observations to existed tracking states by ID
  void AssignTrackObsIdMatch(const SensorObjects &radar_obs,
                             std::vector<std::pair<int, int>> *assignment,
                             std::vector<int> *unassigned_track,
                             std::vector<int> *unassigned_obs);

  void UpdateAssignedTrack(const SensorObjects &radar_obs,
                           const std::vector<std::pair<int, int>> &assignment);

  // update tracking states which fail to find a observation match (set to NULL)
  void UpdateUnassignedTrack(const double &timestamp,
                             std::vector<int> *unassigned_track);

  void DeleteLostTrack();

  void CreateNewTrack(const SensorObjects &radar_obs,
                      std::vector<int> *unassigned_obs);

  double DistanceBetweenObs(const Object &obs1, double timestamp1,
                            const Object &obs2, double timestamp2);

  SensorObjects &GetRadarObs() {
    return radar_obs_;
  }

  std::vector<RadarTrack> &GetTracks() {
    return obs_track_;
  }

 private:
  SensorObjects radar_obs_;
  std::vector<RadarTrack> obs_track_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_RADAR_TRACK_MANAGER_H_
