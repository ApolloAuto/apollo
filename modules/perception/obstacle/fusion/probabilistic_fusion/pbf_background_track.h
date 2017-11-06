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
 
#ifndef ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BACKGROUND_TRACK_H
#define ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BACKGROUND_TRACK_H

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"

namespace apollo {
namespace perception {

class PbfBackgroundTrack {
public:
    explicit PbfBackgroundTrack(PbfSensorObjectPtr obj);

    ~PbfBackgroundTrack();

    /**@brief Update track with sensor object */
    void update_with_sensor_object(PbfSensorObjectPtr obj);

    void update_without_sensor_object(const SensorType& sensor_type,
        const std::string& sensor_id,  double timestamp);

    PbfSensorObjectPtr get_fused_object();

    inline double get_fused_timestamp() const {
        return _fused_timestamp;
    }

    inline int get_track_id() const {
        return _idx;
    }

    inline double get_tracking_period() const {
        return _tracking_period;
    }

    inline bool is_dead() const {
        return _is_dead;
    }

    bool able_to_publish() const;

    static void set_max_invisible_period(double period);

protected:
    PbfSensorObjectPtr    _fused_object;

    /**@brief time stamp of the track*/
    double                _fused_timestamp;

    int                   _age;
    double                _tracking_period;

    /**@brief global track id*/
    int                   _idx;
    double                _invisible_period;

    bool                  _is_dead;

    static double         _s_max_invisible_period;
private:
    PbfBackgroundTrack();
};

typedef std::shared_ptr<PbfBackgroundTrack> PbfBackgroundTrackPtr;

} // namespace perception
} // namespace apollo

#endif
