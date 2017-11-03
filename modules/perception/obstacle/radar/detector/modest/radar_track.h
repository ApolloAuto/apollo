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

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_RADAR_TRACK_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_RADAR_TRACK_H_

#include <memory>
#include <Eigen/Core>
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/common/log.h"
#include "modules/perception/obstacle/radar/detector/modest/radar_define.h"
#include "modules/perception/obstacle/radar/interface/base_filter.h"
#include "modules/perception/obstacle/radar/filter/akf/adaptive_kalman_filter.h"

namespace apollo {
namespace perception {

class RadarTrack {
public:
    RadarTrack();

    RadarTrack(Object &dobs, const double &timestamp);

    RadarTrack(const RadarTrack &track);

    RadarTrack& operator = (const RadarTrack &track);

    ~RadarTrack(){}

    void Prediction(double object_time);

    // update the object after association with a radar obervation
    void SetObsRadar(ObjectPtr obs_radar, const double timestamp);

    // without timestamp, to set the radar observation to NULL
    // (implemented in radar_local_detector.cpp)
    void SetObsRadarWithoutTimestamp(ObjectPtr obs_radar);

    // tracking is considered to be successful if _tracked_times >= 4
    bool ConfirmTrack();

    void IncreaseTrackedTimes();

    int GetTrackedTimes();

    int GetObsId() const;

    ObjectPtr GetObsRadar();

    const ObjectPtr GetObsRadar() const;

    void SetObsRadar(ObjectPtr obs_radar);

    ObjectPtr GetObs();

    const ObjectPtr GetObs() const;

    double GetTimestamp();

    double GetTrackingTime();

    void TrueIdTracked();

    void FalseIdTracked();

    static void SetFilterType(std::string filter_type) {
        s_chosen_filter_ = filter_type;
    }
    
    static void SetTrackedTimesThreshold(const int& threshold) {
        s_tracked_times_threshold_ = threshold;
    }

private:
    static std::string s_chosen_filter_;
    static int s_current_idx_;
    static int s_tracked_times_threshold_;
    int obs_id_;
    double timestamp_;
    ObjectPtr obs_radar_;  // observation from radar
    ObjectPtr obs_;        // track state after filtering
    boost::shared_ptr<BaseFilter> tracker_;    // kalman filter
    int tracked_times_;
    double tracking_time_;
    bool id_tracked_;
};

} // namespace perception
} // namespace apollo

#endif // MODULES_PERCEPTION_OBSTACLE_RADAR_RADAR_TRACK_H_
