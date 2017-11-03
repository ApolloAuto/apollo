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

 #include "modules/perception/obstacle/radar/detector/modest/radar_track_manager.h"

namespace apollo {
namespace perception {

void RadarTrackManager::Process(const SensorObjects &obs_array) {
    radar_obs_array_ = obs_array;
    Update(radar_obs_array_);
}

void RadarTrackManager::Update(SensorObjects &obs_array) {
    std::vector<std::pair<int, int> > assignment;
    std::vector<int> unassigned_track;
    std::vector<int> unassigned_obs;
    AssignTrackObsIdMatch(obs_array, assignment,
        unassigned_track, unassigned_obs);
    UpdateAssignedTrack(obs_array, assignment);
    UpdateUnassignedTrack(obs_array.timestamp, unassigned_track);
    DeleteLostTrack();
    CreateNewTrack(obs_array, unassigned_obs);
}

void RadarTrackManager::AssignTrackObsIdMatch(
        const SensorObjects& obs_array,
        std::vector<std::pair<int, int> > &assignment,
        std::vector<int> &unassigned_track,
        std::vector<int> &unassigned_obs) {
    assignment.resize(obs_track_.size() * 2);
    int assignment_num = 0;
    std::vector<bool> track_used(obs_track_.size(), false);
    std::vector<bool> obs_used(obs_array.objects.size(), false);
    for (size_t i = 0; i < obs_track_.size(); i++) {
        std::shared_ptr<Object> obs;
        obs = obs_track_[i].GetObsRadar();
        if (obs == nullptr) {
            continue;
        }
        double timestamp_track = obs_track_[i].GetTimestamp();
        double timestamp_obs = obs_array.timestamp;
        for (size_t j = 0; j < obs_array.objects.size(); j++) {
            double distance = DistanceBetweenObs(*obs, timestamp_track,
                *(obs_array.objects[j]), timestamp_obs);
            if (obs->track_id == obs_array.objects[j]->track_id &&
                distance < RADAR_TRACK_THRES) {
                assignment[assignment_num++] = std::make_pair(i, j);
                track_used[i] = true;
                obs_used[j] = true;
                obs_track_[i].IncreaseTrackedTimes();
                obs_track_[i].TrueIdTracked();
            }
        }
    }

    assignment.resize(assignment_num);
    unassigned_track.resize(obs_track_.size());
    int unassigned_track_num = 0;
    for (size_t i = 0; i < track_used.size(); i++) {
        if (track_used[i] == false) {
            unassigned_track[unassigned_track_num++] = i;
        }
    }
    unassigned_track.resize(unassigned_track_num);

    unassigned_obs.resize(obs_array.objects.size());
    int unassigned_obs_num = 0;
    for (size_t i = 0; i < obs_used.size(); i++) {
        if (obs_used[i] == false) {
            unassigned_obs[unassigned_obs_num++] = i;
        }
    }
    unassigned_obs.resize(unassigned_obs_num);
}

void RadarTrackManager::UpdateAssignedTrack(
        const SensorObjects& obs_array,
        const std::vector<std::pair<int, int> > &assignment) {
    for (size_t i = 0; i < assignment.size(); i++) {
        obs_track_[assignment[i].first].SetObsRadar(
            obs_array.objects[assignment[i].second], obs_array.timestamp);
    }
}

void RadarTrackManager::UpdateUnassignedTrack(
        const double& timestamp,
        std::vector<int> &unassigned_track) {
    double time_stamp = timestamp;
    for (size_t i = 0; i < unassigned_track.size(); i++) {
        if (obs_track_[unassigned_track[i]].GetObsRadar() != nullptr) {
            double radar_time = obs_track_[unassigned_track[i]].GetTimestamp();
            double time_diff = fabs(time_stamp - radar_time);
            if (time_diff > RADAR_TRACK_TIME_WIN) {
                obs_track_[unassigned_track[i]].SetObsRadarWithoutTimestamp(nullptr);
            }
        }
    }
}

void RadarTrackManager::DeleteLostTrack() {
    int track_num = 0;
    for (size_t i = 0; i < obs_track_.size(); i++) {
        if (obs_track_[i].GetObsRadar() != nullptr) {
            obs_track_[track_num++] = obs_track_[i];
        }
    }
    obs_track_.resize(track_num);
}

void RadarTrackManager::CreateNewTrack(
        const SensorObjects& obs_array,
        std::vector<int>& unassigned_obs) {
    for (size_t i = 0; i < unassigned_obs.size(); i++) {
        obs_track_.push_back(
            RadarTrack(
              *(obs_array.objects[unassigned_obs[i]]), obs_array.timestamp));
    }
}

double RadarTrackManager::DistanceBetweenObs(
    const Object& obs1, double timestamp1,
    const Object& obs2, double timestamp2) {
        double time_diff = timestamp2 - timestamp1;
        double cnt1[3] = {obs1.center[0], obs1.center[1], obs1.center[2]};
        // velocity stored in tracks are relative.
        cnt1[0] += obs1.velocity[0] * time_diff;
        cnt1[1] += obs1.velocity[1] * time_diff;

        double cnt2[3] = {obs2.center[0], obs2.center[1], obs2.center[2]};
        double diff_pos[3];
        diff_pos[0] = cnt2[0] - cnt1[0];
        diff_pos[1] = cnt2[1] - cnt1[1];
        diff_pos[2] = cnt2[2] - cnt1[2];
        double dist_pos = sqrt(pow(diff_pos[0], 2) + pow(diff_pos[1], 2));
        return dist_pos;
}

} // namespace perception
} // namespace apollo
